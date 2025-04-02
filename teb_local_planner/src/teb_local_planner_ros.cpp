/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#include <teb_local_planner/teb_local_planner_ros.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/algorithm/string.hpp>

#include <fstream>
#include <sstream>
#include <ctime>  // for std::time_t, std::strftime
#include <iomanip> // for std::put_time

// MBF return codes
#include <mbf_msgs/ExePathResult.h>

// pluginlib macros
#include <pluginlib/class_list_macros.h>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"

#define SDT_DEAD_RECKONING_IMPLEMENTATION
#include <teb_local_planner/sdt_dead_reckoning.h>


// register this planner both as a BaseLocalPlanner and as a MBF's CostmapController plugin
PLUGINLIB_EXPORT_CLASS(teb_local_planner::TebLocalPlannerROS, nav_core::BaseLocalPlanner)
PLUGINLIB_EXPORT_CLASS(teb_local_planner::TebLocalPlannerROS, mbf_costmap_core::CostmapController)

namespace teb_local_planner
{
  

TebLocalPlannerROS::TebLocalPlannerROS() : costmap_ros_(NULL), tf_(NULL), costmap_model_(NULL),
                                           costmap_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons"),
                                           dynamic_recfg_(NULL), custom_via_points_active_(true), goal_reached_(false), no_infeasible_plans_(0),
                                           last_preferred_rotdir_(RotType::none), initialized_(false)
{
}


TebLocalPlannerROS::~TebLocalPlannerROS()
{
}

void TebLocalPlannerROS::reconfigureCB(TebLocalPlannerReconfigureConfig& config, uint32_t level)
{
  cfg_.reconfigure(config);
  ros::NodeHandle nh("~/" + name_);
  // lock the config mutex externally
  boost::mutex::scoped_lock lock(cfg_.configMutex());

  // create robot footprint/contour model for optimization
  cfg_.robot_model = getRobotFootprintFromParamServer(nh, cfg_);
  planner_->updateRobotModel(cfg_.robot_model);
}

void TebLocalPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  // check if the plugin is already initialized
  if(!initialized_)
  {	
    name_ = name;
    // create Node Handle with name of plugin (as used in move_base for loading)
    ros::NodeHandle nh("~/" + name);
	        
    // get parameters of TebConfig via the nodehandle and override the default config
    cfg_.loadRosParamFromNodeHandle(nh);       
    
    // reserve some memory for obstacles
    obstacles_.reserve(500);
        
    // create visualization instance	
    visualization_ = TebVisualizationPtr(new TebVisualization(nh, cfg_)); 
        
    // create robot footprint/contour model for optimization
    cfg_.robot_model = getRobotFootprintFromParamServer(nh, cfg_);
    
    // create the planner instance
    if (cfg_.hcp.enable_homotopy_class_planning)
    {
      planner_ = PlannerInterfacePtr(new HomotopyClassPlanner(cfg_, &obstacles_, visualization_, &via_points_));
      ROS_INFO("Parallel planning in distinctive topologies enabled.");
    }
    else
    {
      planner_ = PlannerInterfacePtr(new TebOptimalPlanner(cfg_, &obstacles_, visualization_, &via_points_));
      ROS_INFO("Parallel planning in distinctive topologies disabled.");
    }
    
    // init other variables
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.
    
    costmap_model_ = boost::make_shared<base_local_planner::CostmapModel>(*costmap_);

    global_frame_ = costmap_ros_->getGlobalFrameID();

    cfg_.map_frame = global_frame_; // TODO
    robot_base_frame_ = costmap_ros_->getBaseFrameID();
   
    //Initialize a costmap to polygon converter
    if (!cfg_.obstacles.costmap_converter_plugin.empty())
    {
      try
      {
        costmap_converter_ = costmap_converter_loader_.createInstance(cfg_.obstacles.costmap_converter_plugin);
        std::string converter_name = costmap_converter_loader_.getName(cfg_.obstacles.costmap_converter_plugin);
        // replace '::' by '/' to convert the c++ namespace to a NodeHandle namespace
        boost::replace_all(converter_name, "::", "/");
        costmap_converter_->setOdomTopic(cfg_.odom_topic);
        costmap_converter_->initialize(ros::NodeHandle(nh, "costmap_converter/" + converter_name));
        costmap_converter_->setCostmap2D(costmap_);
        
        costmap_converter_->startWorker(ros::Rate(cfg_.obstacles.costmap_converter_rate), costmap_, cfg_.obstacles.costmap_converter_spin_thread);
        ROS_INFO_STREAM("Costmap conversion plugin " << cfg_.obstacles.costmap_converter_plugin << " loaded.");        
      }
      catch(pluginlib::PluginlibException& ex)
      {
        ROS_WARN("The specified costmap converter plugin cannot be loaded. All occupied costmap cells are treaten as point obstacles. Error message: %s", ex.what());
        costmap_converter_.reset();
      }
    }
    else 
      ROS_INFO("No costmap conversion plugin specified. All occupied costmap cells are treaten as point obstacles.");
  
    
    // Get footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
    footprint_spec_ = costmap_ros_->getRobotFootprint();
    costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);    
    
    // init the odom helper to receive the robot's velocity from odom messages
    odom_helper_.setOdomTopic(cfg_.odom_topic);

    // setup dynamic reconfigure
    dynamic_recfg_ = boost::make_shared< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> >(nh);
    dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(&TebLocalPlannerROS::reconfigureCB, this, _1, _2);
    dynamic_recfg_->setCallback(cb);
    
    // validate optimization footprint and costmap footprint
    validateFootprints(cfg_.robot_model->getInscribedRadius(), robot_inscribed_radius_, cfg_.obstacles.min_obstacle_dist);
        
    // setup callback for custom obstacles
    custom_obst_sub_ = nh.subscribe("obstacles", 1, &TebLocalPlannerROS::customObstacleCB, this);

    // setup callback for custom via-points
    via_points_sub_ = nh.subscribe("via_points", 1, &TebLocalPlannerROS::customViaPointsCB, this);
    
    // initialize failure detector
    ros::NodeHandle nh_move_base("~");
    double controller_frequency = 5;
    nh_move_base.param("controller_frequency", controller_frequency, controller_frequency);
    failure_detector_.setBufferLength(std::round(cfg_.recovery.oscillation_filter_duration*controller_frequency));
    
    // set initialized flag
    initialized_ = true;

    ROS_DEBUG("teb_local_planner plugin initialized.");
  }
  else
  {
    ROS_WARN("teb_local_planner has already been initialized, doing nothing.");
  }
}

bool TebLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  // check if plugin is initialized
  if(!initialized_)
  {
    ROS_ERROR("teb_local_planner has not been initialized, please call initialize() before using this planner");
    return false;
  }
  
  // store the global plan
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  // // 매번 호출될 때마다 고유한 파일 이름 생성 (예: /tmp/global_plan_초_나노초.txt)
  // ros::Time now = ros::Time::now();
  // std::stringstream ss;
  // ss << "home/glab/tmp/global_plan_" << now.sec << "_" << now.nsec << ".txt";
  // std::string filename = ss.str();

  // std::ofstream outfile(filename.c_str());
  // if (outfile.is_open())
  // {
  //   for (const auto& pose_stamped : orig_global_plan)
  //   {
  //     const geometry_msgs::Pose& pose = pose_stamped.pose;
  //     // position과 orientation 정보를 공백으로 구분하여 저장 (각 행은 하나의 Pose)
  //     outfile << pose.position.x << " " << pose.position.y << " " << pose.position.z << " ";
  //     outfile << pose.orientation.x << " " << pose.orientation.y << " " << pose.orientation.z << " " << pose.orientation.w << "\n";
  //   }
  //   outfile.close();
  //   ROS_INFO("Global plan saved to %s", filename.c_str());
  // }
  // else
  // {
  //   ROS_ERROR("Unable to open file %s for writing global plan", filename.c_str());
  // }
  
  // we do not clear the local planner here, since setPlan is called frequently whenever the global planner updates the plan.
  // the local planner checks whether it is required to reinitialize the trajectory or not within each velocity computation step.  
            
  // reset goal_reached_ flag
  goal_reached_ = false;
  
  return true;
}


// bool TebLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
// {
//   static bool manual_plan_set_ = false; // 수동 Plan 설정 여부 플래그

//   // check if plugin is initialized
//   if (!initialized_)
//   {
//     ROS_ERROR("teb_local_planner has not been initialized, please call initialize() before using this planner");
//     return false;
//   }

//   if (!manual_plan_set_)
//   {
//     // 수동으로 Global Plan 설정
//     global_plan_.clear();
//     geometry_msgs::PoseStamped pose;

//     // Example manual plan (사용자가 정의한 Plan)
//     std::vector<std::pair<double, double>> manual_plan = {
//     {-0.426784, 4.42194}, {-0.461211, 4.41846}, {-0.48614, 4.41657}, {-0.511011, 4.41404}, 
//     {-0.535795, 4.41076}, {-0.560446, 4.40659}, {-0.584909, 4.40144}, {-0.60911, 4.39517}, 
//     {-0.632978, 4.38773}, {-0.65641, 4.37902}, {-0.679485, 4.3694}, {-0.702521, 4.35969}, 
//     {-0.725526, 4.3499}, {-0.748472, 4.33998}, {-0.771368, 4.32994}, {-0.794212, 4.31978}, 
//     {-0.817076, 4.30967}, {-0.839912, 4.2995}, {-0.862722, 4.28926}, {-0.885549, 4.27907}, 
//     {-0.90837, 4.26886}, {-0.931262, 4.25881}, {-0.954282, 4.24906}, {-0.977492, 4.23977}, 
//     {-1.00091, 4.23103}, {-1.02457, 4.22296}, {-1.04826, 4.21495}, {-1.07193, 4.20693}, 
//     {-1.09559, 4.19884}, {-1.11923, 4.1907}, {-1.14284, 4.18247}, {-1.16641, 4.17416}, 
//     {-1.18998, 4.16582}, {-1.21354, 4.15746}, {-1.23709, 4.14907}, {-1.26063, 4.14064}, 
//     {-1.28415, 4.13216}, {-1.30764, 4.12363}, {-1.33113, 4.11507}, {-1.35461, 4.10648}, 
//     {-1.3781, 4.09792}, {-1.40155, 4.08924}, {-1.425, 4.08059}, {-1.44845, 4.07193}, 
//     {-1.47192, 4.06332}, {-1.49545, 4.05485}, {-1.51907, 4.04666}, {-1.54281, 4.03883}, 
//     {-1.5667, 4.03146}, {-1.59075, 4.02463}, {-1.61483, 4.01791}, {-1.63882, 4.01089}, 
//     {-1.66268, 4.00342}, {-1.68635, 3.99537}, {-1.70974, 3.98656}, {-1.73278, 3.97684}, 
//     {-1.75561, 3.96667}, {-1.77847, 3.95654}, {-1.80138, 3.94653}, {-1.82432, 3.9366}, 
//     {-1.84763, 3.92756}, {-1.86978, 3.91596}, {-1.89234, 3.90519}, {-1.91683, 3.90019}, 
//     {-1.94171, 3.89772}, {-1.96671, 3.89803}, {-1.9875, 3.89191}, {-2.01249, 3.88999}, 
//     {-2.03608, 3.88888}, {-2.05962, 3.88777}, {-2.08307, 3.88555}, {-2.10711, 3.88333}, 
//     {-2.13183, 3.87999}, {-2.15671, 3.87555}, {-2.1817, 3.87302}, {-2.2067, 3.87263}, 
//     {-2.2317, 3.87275}, {-2.25669, 3.87222}, {-2.28167, 3.87222}, {-2.30657, 3.87222}, 
//     {-2.32689, 3.87222}, {-2.35123, 3.87222}, {-2.36351, 3.87222}, {-2.37678, 3.87194}, 
//     {-2.38987, 3.87333}, {-2.40788, 3.87444}, {-2.43001, 3.87555}, {-2.45193, 3.87444}, 
//     {-2.47358, 3.87333}, {-2.48393, 3.87444}, {-2.50747, 3.8748}, {-2.53208, 3.87918}, 
//     {-2.55666, 3.88372}, {-2.5811, 3.88902}, {-2.6047, 3.89727}, {-2.62736, 3.90782}, 
//     {-2.64964, 3.91916}, {-2.67338, 3.92701}, {-2.69682, 3.9357}, {-2.72011, 3.94478}, 
//     {-2.74326, 3.95422}, {-2.7663, 3.96393}, {-2.78945, 3.97336}, {-2.8124, 3.98327}, 
//     {-2.83519, 3.99354}, {-2.85785, 4.00411}, {-2.88042, 4.01487}, {-2.90303, 4.02553}, 
//     {-2.92551, 4.03648}, {-2.94787, 4.04766}, {-2.97014, 4.059}, {-2.99235, 4.0705}, 
//     {-3.01479, 4.08151}, {-3.03721, 4.09256}, {-3.05952, 4.10385}, {-3.08179, 4.1152}, 
//     {-3.10503, 4.12442}, {-3.12841, 4.13329}, {-3.15184, 4.14201}, {-3.17532, 4.15057}, 
//     {-3.19844, 4.16009}, {-3.22125, 4.17033}, {-3.24446, 4.17962}, {-3.26769, 4.18885}, 
//     {-3.29094, 4.19804}, {-3.3142, 4.20721}, {-3.33745, 4.21639}, {-3.36073, 4.22551}, 
//     {-3.38405, 4.23452}, {-3.4073, 4.24372}, {-3.4305, 4.25301}, {-3.45364, 4.26248}, 
//     {-3.47678, 4.27194}, {-3.52678, 4.32194}, {-3.495294, 4.35859}};

//     // Manual Plan 추가
//     for (const auto& point : manual_plan)
//     {
//       pose.header.frame_id = "map"; 
//       pose.header.stamp = ros::Time::now(); 
//       pose.pose.position.x = point.first;
//       pose.pose.position.y = point.second;
//       pose.pose.position.z = 0.0; // Assume 2D plan
//       pose.pose.orientation.x = 0.0;
//       pose.pose.orientation.y = 0.0;
//       pose.pose.orientation.z = 0.0;
//       pose.pose.orientation.w = 1.0; // Default orientation
//       global_plan_.push_back(pose);
//     }                         

//     ROS_INFO("Manual global plan has been set.");

//     manual_plan_set_ = true; // 플래그 설정
//   }
//   else
//   {
//     ROS_WARN("Global plan update is ignored since a manual plan has been set.");
//   }

//   // reset goal_reached_ flag
//   goal_reached_ = false;

//   return true;
// }

bool TebLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  auto start = std::chrono::high_resolution_clock::now();

  ROS_DEBUG("computeVelocityCommands");
  std::string dummy_message;
  geometry_msgs::PoseStamped dummy_pose;
  geometry_msgs::TwistStamped dummy_velocity, cmd_vel_stamped;
  uint32_t outcome = computeVelocityCommands(dummy_pose, dummy_velocity, cmd_vel_stamped, dummy_message);
  cmd_vel = cmd_vel_stamped.twist;

  // 시간 측정 종료
  auto end = std::chrono::high_resolution_clock::now();

  // 경과 시간 계산 (마이크로초 단위)
  auto duration = std::chrono::duration<double, std::milli>(end - start).count();
  
  // 파일에 저장
  std::ofstream outFile("/home/glab/execution_time.txt", std::ios::app); // 파일을 append 모드로 열기
  if (outFile.is_open()) {
      outFile << "Execution time: " << duration << " ms" << std::endl;
      outFile.close();
  } else {
      std::cerr << "Failed to open file for writing." << std::endl;
  }
  
  // 콘솔 출력
  std::cout << "Execution time: " << duration << " ms" << std::endl;
  
  return outcome == mbf_msgs::ExePathResult::SUCCESS;
}

uint32_t TebLocalPlannerROS::computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                                     const geometry_msgs::TwistStamped& velocity,
                                                     geometry_msgs::TwistStamped &cmd_vel,
                                                     std::string &message)
{

  ROS_DEBUG("computeVelocityCommands for real plan");
  // check if plugin initialized
  if(!initialized_)
  {
    ROS_ERROR("teb_local_planner has not been initialized, please call initialize() before using this planner");
    message = "teb_local_planner has not been initialized";
    return mbf_msgs::ExePathResult::NOT_INITIALIZED;
  }

  static uint32_t seq = 0;
  cmd_vel.header.seq = seq++;
  cmd_vel.header.stamp = ros::Time::now();
  cmd_vel.header.frame_id = robot_base_frame_;
  cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
  goal_reached_ = false;  
  
  // Get robot pose
  geometry_msgs::PoseStamped robot_pose;
  costmap_ros_->getRobotPose(robot_pose);
  robot_pose_ = PoseSE2(robot_pose.pose);
    
  // Get robot velocity
  geometry_msgs::PoseStamped robot_vel_tf;
  odom_helper_.getRobotVel(robot_vel_tf);
  robot_vel_.linear.x = robot_vel_tf.pose.position.x;
  robot_vel_.linear.y = robot_vel_tf.pose.position.y;
  robot_vel_.angular.z = tf2::getYaw(robot_vel_tf.pose.orientation);
  
  // 1. prune global plan to cut off parts of the past (spatially before the robot)
  pruneGlobalPlan(*tf_, robot_pose, global_plan_, cfg_.trajectory.global_plan_prune_distance);

  // 2. Transform global plan to the frame of interest (w.r.t. the local costmap)
  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  int goal_idx;
  geometry_msgs::TransformStamped tf_plan_to_global;
  if (!transformGlobalPlan(*tf_, global_plan_, robot_pose, *costmap_, global_frame_, cfg_.trajectory.max_global_plan_lookahead_dist, 
                           transformed_plan, &goal_idx, &tf_plan_to_global))
  {
    ROS_WARN("Could not transform the global plan to the frame of the controller");
    message = "Could not transform the global plan to the frame of the controller";
    return mbf_msgs::ExePathResult::INTERNAL_ERROR;
  }

  // 3. update via-points container
  if (!custom_via_points_active_)
    updateViaPointsContainer(transformed_plan, cfg_.trajectory.global_plan_viapoint_sep);
  else
    updateCustomViaPointsContainer(transformed_plan, *costmap_);

  nav_msgs::Odometry base_odom;
  odom_helper_.getOdom(base_odom);

  // 4. check if global goal is reached
  geometry_msgs::PoseStamped global_goal;
  tf2::doTransform(global_plan_.back(), global_goal, tf_plan_to_global);
  double dx = global_goal.pose.position.x - robot_pose_.x();
  double dy = global_goal.pose.position.y - robot_pose_.y();
  double delta_orient = g2o::normalize_theta( tf2::getYaw(global_goal.pose.orientation) - robot_pose_.theta() );
  if(fabs(std::sqrt(dx*dx+dy*dy)) < cfg_.goal_tolerance.xy_goal_tolerance
    && fabs(delta_orient) < cfg_.goal_tolerance.yaw_goal_tolerance
    && (!cfg_.goal_tolerance.complete_global_plan || via_points_.size() == 0)
    && (base_local_planner::stopped(base_odom, cfg_.goal_tolerance.theta_stopped_vel, cfg_.goal_tolerance.trans_stopped_vel)
        || cfg_.goal_tolerance.free_goal_vel))
  {
    goal_reached_ = true;
    return mbf_msgs::ExePathResult::SUCCESS;
  }

  // check if we should enter any backup mode and apply settings
  configureBackupModes(transformed_plan, goal_idx);
  
    
  // Return false if the transformed global plan is empty
  if (transformed_plan.empty())
  {
    ROS_WARN("Transformed plan is empty. Cannot determine a local plan.");
    message = "Transformed plan is empty";
    return mbf_msgs::ExePathResult::INVALID_PATH;
  }
              
  // Get current goal point (last point of the transformed plan)
  robot_goal_.x() = transformed_plan.back().pose.position.x;
  robot_goal_.y() = transformed_plan.back().pose.position.y;
  // Overwrite goal orientation if needed
  if (cfg_.trajectory.global_plan_overwrite_orientation)
  {
    robot_goal_.theta() = estimateLocalGoalOrientation(global_plan_, transformed_plan.back(), goal_idx, tf_plan_to_global);
    // overwrite/update goal orientation of the transformed plan with the actual goal (enable using the plan as initialization)
    tf2::Quaternion q;
    q.setRPY(0, 0, robot_goal_.theta());
    tf2::convert(q, transformed_plan.back().pose.orientation);
  }  
  else
  {
    robot_goal_.theta() = tf2::getYaw(transformed_plan.back().pose.orientation);
  }

  // overwrite/update start of the transformed plan with the actual robot position (allows using the plan as initial trajectory)
  if (transformed_plan.size()==1) // plan only contains the goal
  {
    transformed_plan.insert(transformed_plan.begin(), geometry_msgs::PoseStamped()); // insert start (not yet initialized)
  }
  transformed_plan.front() = robot_pose; // update start
    
  // clear currently existing obstacles
  obstacles_.clear();
  
  // Update obstacle container with costmap information or polygons provided by a costmap_converter plugin
  if (costmap_converter_)
  {
    ROS_DEBUG("updateObstacleContainerWithCostmapConverter");
    updateObstacleContainerWithCostmapConverter();
  }
  else
  {
    ROS_DEBUG("updateObstacleContainerWithCostmap");
    updateObstacleContainerWithCostmap();
  }
  // also consider custom obstacles (must be called after other updates, since the container is not cleared)
  updateObstacleContainerWithCustomObstacles();
  
    
  // Do not allow config changes during the following optimization step
  boost::mutex::scoped_lock cfg_lock(cfg_.configMutex());

  int look_ahead_idx = cfg_.trajectory.feasibility_check_no_poses;
  double inscribed_radius = robot_inscribed_radius_;

  ROS_DEBUG("transformed plan size : %d", transformed_plan.size());
  
  // Now perform the actual planning
  // bool success = planner_->plan(robot_pose_, robot_goal_, robot_vel_, cfg_.goal_tolerance.free_goal_vel); // straight line init
  ROS_DEBUG("planning based transformed plan");

  // bool success = planner_->plan(costmap_model_.get(), footprint_spec_, transformed_plan, robot_inscribed_radius_, robot_circumscribed_radius, &robot_vel_, cfg_.goal_tolerance.free_goal_vel);
  // local planning (using TEB)
  auto start0 = std::chrono::high_resolution_clock::now();
  bool success = planner_->plan(transformed_plan, &robot_vel_, cfg_.goal_tolerance.free_goal_vel);
  // 시간 측정 종료
  auto end0 = std::chrono::high_resolution_clock::now();

  // 경과 시간 계산 (마이크로초 단위)
  auto duration0 = std::chrono::duration<double, std::milli>(end0 - start0).count();

  std::ofstream outFile("/home/glab/execution_time.txt", std::ios::app);   
  if (outFile.is_open()) {
      outFile << "Optimization time: " << duration0 << " ms" << std::endl;
      outFile.close();
  } else {
      std::cerr << "Failed to open file for writing." << std::endl;
  }
    
  // 콘솔 출력
  std::cout << "Optimization time: " << duration0 << " ms" << std::endl;

  ROS_DEBUG("Plan result: %s", success ? "successful" : "failed");

  if (!success)
  {
    planner_->clearPlanner(); // force reinitialization for next time
    ROS_WARN("teb_local_planner was not able to obtain a local plan for the current setting.");
    
    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = ros::Time::now();
    last_cmd_ = cmd_vel.twist;
    message = "teb_local_planner was not able to obtain a local plan";
    return mbf_msgs::ExePathResult::NO_VALID_CMD;
  }

  ROS_DEBUG("teb_local_planner was able to obtain a local plan for the current setting.");  

  // Check for divergence
  ROS_DEBUG("Check for divergence");

  if (planner_->hasDiverged())
  {
    cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;

    // Reset everything to start again with the initialization of new trajectories.
    planner_->clearPlanner();
    ROS_WARN_THROTTLE(1.0, "TebLocalPlannerROS: the trajectory has diverged. Resetting planner...");

    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = ros::Time::now();
    last_cmd_ = cmd_vel.twist;
    return mbf_msgs::ExePathResult::NO_VALID_CMD;
  }

  ROS_DEBUG("the trajectory has not diverged.");
         
  // Check feasibility (but within the first few states only)

  if(cfg_.robot.is_footprint_dynamic)
  {
    // Update footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
    footprint_spec_ = costmap_ros_->getRobotFootprint();
    costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);
  }
  ROS_DEBUG("Check for feasibility");

  bool feasible = planner_->isTrajectoryFeasible(costmap_model_.get(), footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius, cfg_.trajectory.feasibility_check_no_poses, cfg_.trajectory.feasibility_check_lookahead_distance);
  if (!feasible)
  {
    cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;

    // now we reset everything to start again with the initialization of new trajectories.
    planner_->clearPlanner();
    ROS_WARN("TebLocalPlannerROS: trajectory is not feasible. Resetting planner...");
    
    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = ros::Time::now();
    last_cmd_ = cmd_vel.twist;
    message = "teb_local_planner trajectory is not feasible";
    return mbf_msgs::ExePathResult::NO_VALID_CMD;
  }

  // Get the velocity command for this sampling interval
  ROS_DEBUG("Get the velocity command");
  if (!planner_->getVelocityCommand(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z, cfg_.trajectory.control_look_ahead_poses))

  {
    planner_->clearPlanner();
    ROS_WARN("TebLocalPlannerROS: velocity command invalid. Resetting planner...");
    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = ros::Time::now();
    last_cmd_ = cmd_vel.twist;
    message = "teb_local_planner velocity command invalid";
    return mbf_msgs::ExePathResult::NO_VALID_CMD;
  }
  
  // Saturate velocity, if the optimization results violates the constraints (could be possible due to soft constraints).
  saturateVelocity(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z,
                   cfg_.robot.max_vel_x, cfg_.robot.max_vel_y, cfg_.robot.max_vel_trans, cfg_.robot.max_vel_theta, 
                   cfg_.robot.max_vel_x_backwards);

  // convert rot-vel to steering angle if desired (carlike robot).
  // The min_turning_radius is allowed to be slighly smaller since it is a soft-constraint
  // and opposed to the other constraints not affected by penalty_epsilon. The user might add a safety margin to the parameter itself.
  if (cfg_.robot.cmd_angle_instead_rotvel)
  {
    cmd_vel.twist.angular.z = convertTransRotVelToSteeringAngle(cmd_vel.twist.linear.x, cmd_vel.twist.angular.z,
                                                                cfg_.robot.wheelbase, 0.95*cfg_.robot.min_turning_radius);
    if (!std::isfinite(cmd_vel.twist.angular.z))
    {
      cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
      last_cmd_ = cmd_vel.twist;
      planner_->clearPlanner();
      ROS_WARN("TebLocalPlannerROS: Resulting steering angle is not finite. Resetting planner...");
      ++no_infeasible_plans_; // increase number of infeasible solutions in a row
      time_last_infeasible_plan_ = ros::Time::now();
      message = "teb_local_planner steering angle is not finite";
      return mbf_msgs::ExePathResult::NO_VALID_CMD;
    }
  }
  
  // a feasible solution should be found, reset counter
  no_infeasible_plans_ = 0;
  
  // store last command (for recovery analysis etc.)
  last_cmd_ = cmd_vel.twist;
  
  // Now visualize everything    
  planner_->visualize();
  visualization_->publishObstacles(obstacles_, costmap_->getResolution());
  visualization_->publishCustomViaPoints(via_points_);
  visualization_->publishGlobalPlan(global_plan_);
  // std::cout << "Press Enter to continue..." << std::endl;
  // std::cin.get();
  return mbf_msgs::ExePathResult::SUCCESS;
}


bool TebLocalPlannerROS::isGoalReached()
{
  if (goal_reached_)
  {
    ROS_INFO("GOAL Reached!");
    planner_->clearPlanner();
    return true;
  }
  return false;
}

//last version 

// 2D 포인트 구조체 (파일 범위에 정의)
struct Point2D {
  double x, y;
};

// nanoflann용 포인트 클라우드 래퍼 (파일 범위에 정의)
// local class 내에서 template member 함수 선언이 불가능하므로, 여기서 정의합니다.
struct PointCloud2D {
  std::vector<Point2D> pts;

  inline size_t kdtree_get_point_count() const { return pts.size(); }
  inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
    return (dim == 0) ? pts[idx].x : pts[idx].y;
  }
  // nanoflann에서는 이 함수가 옵션이므로, 항상 false를 반환하면 됩니다.
  template <class BBOX>
  bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }
};

typedef nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, PointCloud2D>,
    PointCloud2D,
    2
> KDTree2D;

std::unique_ptr<KDTree2D> obstacle_kd_tree_;
PointCloud2D obstacle_cloud_;

// Medial Ball Heuristic Algorithm
std::vector<std::pair<geometry_msgs::Point, double>> TebLocalPlannerROS::detectNarrowPassages(const std::vector<geometry_msgs::PoseStamped>& transformed_plan, const costmap_2d::Costmap2D& costmap)
{
  std::vector<std::pair<geometry_msgs::Point, double>> medial_axis_point;

  auto start1 = std::chrono::high_resolution_clock::now();

  std::vector<geometry_msgs::Point> samples = generateSamples(transformed_plan, *costmap_);
  // 시간 측정 종료
  auto end1 = std::chrono::high_resolution_clock::now();

  // 경과 시간 계산 (마이크로초 단위)
  auto duration1 = std::chrono::duration<double, std::milli>(end1 - start1).count();
    
  std::ofstream outFile("/home/glab/execution_time.txt", std::ios::app); 
  if (outFile.is_open()) {
      outFile << "Generate Samples Execution time: " << duration1 << " ms" << std::endl;
      outFile.close();
  } else {
      std::cerr << "Failed to open file for writing." << std::endl;
  }
    
  // 콘솔 출력
  std::cout << "Generate Samples Execution time: " << duration1 << " ms" << std::endl;
  visualization_->visualizeSamples(samples);

  double obst_radius = 0.6;

  std::vector<geometry_msgs::Point> narrow_points; // Store samples with obstacles

  for (const auto& sample : samples)
  {
    auto obstacles_in_circle = getObstaclePointsInCircle(sample, obst_radius);

    if (obstacles_in_circle)
    {
      visualization_->visualizeNarrowSpace(sample, obst_radius);
      narrow_points.push_back(sample);
    }
  }

  double goal_threshold = 0.3;

  geometry_msgs::Point robot_position;
  robot_position.x = robot_pose_.x();
  robot_position.y = robot_pose_.y();
  double yaw = robot_pose_.theta();  // 직접 theta 값을 사용
  // 로봇의 진행 방향 벡터: (cos(yaw), sin(yaw))
  double heading_x = cos(yaw);
  double heading_y = sin(yaw);

  // 장애물이 포함된 샘플로 Medial Ball 생성
  for (const auto& point : narrow_points)
  {
    auto start2 = std::chrono::high_resolution_clock::now();
    
    auto medial_result = findMedialBallRadius(point, *costmap_);
    double medial_radius = medial_result.second;
    geometry_msgs::Point final_center = medial_result.first;

   // 시간 측정 종료
    auto end2 = std::chrono::high_resolution_clock::now();

    // 경과 시간 계산 (마이크로초 단위)
    auto duration2 = std::chrono::duration<double, std::milli>(end2 - start2).count();
      
    // 파일에 저장
    std::ofstream outFile("/home/glab/execution_time.txt", std::ios::app); // 파일을 append 모드로 열기
    if (outFile.is_open()) {
        outFile << "Create Medial Ball Execution time: " << duration2 << " ms" << std::endl;
        outFile.close();
    } else {
        std::cerr << "Failed to open file for writing." << std::endl;
    }
      
    // 콘솔 출력
    std::cout << "Create Medial Ball Execution time: " << duration2 << " ms" << std::endl;
    ROS_INFO("Medial Ball Center: (%.3f, %.3f), Radius: %.3f",final_center.x, final_center.y, medial_radius);


    // 로봇과 medial point 사이의 벡터 계산
    double dx = final_center.x - robot_position.x;
    double dy = final_center.y - robot_position.y;
    // 진행 방향 벡터와의 내적 계산 (로봇 앞쪽인지 확인)
    double dot = dx * heading_x + dy * heading_y;
    // 내적이 음수이면 medial point는 로봇의 뒷쪽에 위치하므로 건너뜁니다.

    if (dot < 0)
      continue;

    double distance_to_goal = euclideanDistance(final_center, transformed_plan.back().pose.position);

    ROS_INFO("distance to goal : %lf", distance_to_goal);

    // threshold 이상인 경우 추가하지 않음
    if (medial_radius < thre && medial_radius >= 0.05 && distance_to_goal > goal_threshold)
    {
      medial_axis_point.emplace_back(final_center, medial_radius);
      visualization_->visualizeMedialBall(final_center, medial_radius);
    }

  }

  return medial_axis_point;
}

double TebLocalPlannerROS::calculateAngle(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, const geometry_msgs::Point& center)
{
  double vector1_x = p1.x - center.x;
  double vector1_y = p1.y - center.y;
  double vector2_x = p2.x - center.x;
  double vector2_y = p2.y - center.y;

  double dot_product = vector1_x * vector2_x + vector1_y * vector2_y;
  double magnitude1 = std::sqrt(vector1_x * vector1_x + vector1_y * vector1_y);
  double magnitude2 = std::sqrt(vector2_x * vector2_x + vector2_y * vector2_y);

  if (magnitude1 > 0 && magnitude2 > 0)
  {
    double cosine_angle = dot_product / (magnitude1 * magnitude2);

    //범위 제한
    if (cosine_angle < -1.0)
      cosine_angle = -1.0;
    else if (cosine_angle > 1.0)
      cosine_angle = 1.0;

    return std::acos(cosine_angle); // 각도 (라디안)
  }
  return 0.0;
}

// helper func: 주어진 grid 좌표 (mx, my)가 costmap 범위 내에 있고 장애물(또는 정보 없음)인지 검사.
// if obstalce  1, or return 0.
int TebLocalPlannerROS::checkAndCount(int mx, int my)
{
  if (mx < 0 || my < 0 || 
      mx >= static_cast<int>(costmap_->getSizeInCellsX()) || 
      my >= static_cast<int>(costmap_->getSizeInCellsY()))
    return 0;
    
  unsigned char cost = costmap_->getCost(mx, my);
  return (cost == costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::NO_INFORMATION) ? 1 : 0;
}

bool TebLocalPlannerROS::getObstaclePointsInCircle(const geometry_msgs::Point& center, double radius)
{
  int min_obstacles = 2;  // narrow하다고 판단하기 위한 최소 장애물 점 개수

  // 중심 좌표를 costmap의 grid index로 변환
  unsigned int center_mx, center_my;
  if (!costmap_->worldToMap(center.x, center.y, center_mx, center_my))
    return false;  // center가 costmap 범위 밖이면 그냥 false
  
  // 반경을 grid 단위(셀 수)로 변환 (해상도로 나누어 정수화)
  double resolution = costmap_->getResolution();
  int grid_radius = std::max(1, static_cast<int>(radius / resolution));

  int obstacleCount = 0;

  // Bresenham의 원 알고리즘을 사용하여 원 둘레상의 셀을 계산
  int x = grid_radius;
  int y = 0;
  int err = 1 - x;  // 초기 결정 변수

  while (y <= x) {
    // 원의 8분할 대칭 영역에 해당하는 셀들을 검사
    obstacleCount += checkAndCount(center_mx + x, center_my + y);
    obstacleCount += checkAndCount(center_mx + y, center_my + x);
    obstacleCount += checkAndCount(center_mx - x, center_my + y);
    obstacleCount += checkAndCount(center_mx - y, center_my + x);
    obstacleCount += checkAndCount(center_mx - x, center_my - y);
    obstacleCount += checkAndCount(center_mx - y, center_my - x);
    obstacleCount += checkAndCount(center_mx + x, center_my - y);
    obstacleCount += checkAndCount(center_mx + y, center_my - x);

    if (obstacleCount >= min_obstacles)
      return true;  // 충분한 장애물이 있으면 바로 true 반환

    y++;
    if (err < 0) {
      err += 2 * y + 1;
    } else {
      x--;
      err += 2 * (y - x + 1);
    }
  }

  return (obstacleCount >= min_obstacles);
}

double TebLocalPlannerROS::euclideanDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
{
  return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

std::vector<geometry_msgs::Point> TebLocalPlannerROS::generateSamples(
  const std::vector<geometry_msgs::PoseStamped>& transformed_plan,
  const costmap_2d::Costmap2D& costmap)
{
PointCloud2D cloud; // transformed_plan의 x,y 좌표를 kd-tree용 포인트 클라우드로 변환
cloud.pts.reserve(transformed_plan.size());
for (const auto& pose : transformed_plan)
  cloud.pts.push_back({ pose.pose.position.x, pose.pose.position.y });

// nanoflann 2D kd-tree 구축
typedef nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, PointCloud2D>,
    PointCloud2D,
    2 /* dimension */
> KDTree2D;
KDTree2D kd_tree(2, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
kd_tree.buildIndex();

// Costmap 정보 및 난수 생성기 초기화
std::vector<geometry_msgs::Point> samples;
double origin_x = costmap.getOriginX();
double origin_y = costmap.getOriginY();
double resolution = costmap.getResolution();
unsigned int width = costmap.getSizeInCellsX();
unsigned int height = costmap.getSizeInCellsY();
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_int_distribution<> dis_x(0, width - 1);
std::uniform_int_distribution<> dis_y(0, height - 1);

// 샘플링 파라미터
double lambda = 1.0;               // 지수 편향 감쇠 계수
double threshold_distance = 0.2;
double threshold_distance_sq = threshold_distance * threshold_distance;

// 원하는 샘플 개수(num_samples)는 클래스 멤버 또는 상수로 정의되어 있다고 가정
while (samples.size() < num_samples)
{
  int mx = dis_x(gen);
  int my = dis_y(gen);

  // costmap에서 FREE_SPACE 셀만 사용
  if (costmap.getCost(mx, my) == costmap_2d::FREE_SPACE)
  {
    geometry_msgs::Point sample;
    sample.x = origin_x + mx * resolution;
    sample.y = origin_y + my * resolution;

    // kd-tree로 sample에 대해 최근접 이웃 검색 (2D)
    double query_pt[2] = { sample.x, sample.y };
    size_t nearest_idx;
    double out_dist_sq;
    nanoflann::KNNResultSet<double> resultSet(1);
    resultSet.init(&nearest_idx, &out_dist_sq);
    kd_tree.findNeighbors(resultSet, query_pt, nanoflann::SearchParameters(10));

    // 글로벌 경로와의 제곱 거리가 임계값 이내이면 채택
    if (out_dist_sq <= threshold_distance_sq)
    {
      double d_G = std::sqrt(out_dist_sq);  // 실제 거리 계산
      double sampling_weight = lambda * std::exp(-lambda * d_G);
      if (std::uniform_real_distribution<>(0, 1)(gen) < sampling_weight)
        samples.push_back(sample);
    }
  }
}

return samples;
}

void TebLocalPlannerROS::updateObstacleKDTree()
{
  unsigned int size_x = costmap_->getSizeInCellsX();
  unsigned int size_y = costmap_->getSizeInCellsY();
  double origin_x = costmap_->getOriginX();
  double origin_y = costmap_->getOriginY();
  double resolution = costmap_->getResolution();
  obstacle_cloud_.pts.clear();
  for (unsigned int i = 0; i < size_x; ++i) {
    for (unsigned int j = 0; j < size_y; ++j) {
      unsigned char cost = costmap_->getCost(i, j);
      if (cost == costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::NO_INFORMATION) {
        double wx = origin_x + (i + 0.5) * resolution;
        double wy = origin_y + (j + 0.5) * resolution;
        obstacle_cloud_.pts.push_back({ wx, wy });
      }
    }
  }
  obstacle_kd_tree_.reset(new KDTree2D(2, obstacle_cloud_, nanoflann::KDTreeSingleIndexAdaptorParams(10)));
  obstacle_kd_tree_->buildIndex();
  ROS_INFO("Obstacle kd-tree updated with %lu points", obstacle_cloud_.pts.size());
}

bool TebLocalPlannerROS::isObstacleAtPoint(double x, double y, double search_radius)
{
  double query_pt[2] = { x, y };
  std::vector<nanoflann::ResultItem<unsigned int, double>> ret_matches;
  nanoflann::SearchParameters params;
  size_t num = obstacle_kd_tree_->radiusSearch(query_pt, search_radius * search_radius, ret_matches, params);
  return (num > 0);
}

//grid base search

std::pair<geometry_msgs::Point, double> TebLocalPlannerROS::findMedialBallRadius(
  const geometry_msgs::Point& point, const costmap_2d::Costmap2D& costmap)
{
    // 1. costmap info
    unsigned int map_width = costmap.getSizeInCellsX();
    unsigned int map_height = costmap.getSizeInCellsY();
    double resolution = costmap.getResolution();
    double origin_x = costmap.getOriginX();
    double origin_y = costmap.getOriginY();
    const unsigned char* costmap_data = costmap.getCharMap();


    // 2. distance field 생성
    std::vector<float> distance_field(map_width * map_height, std::numeric_limits<float>::infinity());
    sdt_dead_reckoning(map_width, map_height, 253, costmap_data, distance_field.data());



    // 3. 각 sample point에 대해 projection 수행
    geometry_msgs::Point medial_center = performMedialAxisClimb(point, *costmap_, distance_field, resolution, origin_x, origin_y);
        
    int grid_x = static_cast<int>((medial_center.x - origin_x) / resolution);
    int grid_y = static_cast<int>((medial_center.y - origin_y) / resolution);
    int idx = grid_x + grid_y * map_width;
    double radius = distance_field[idx]* resolution;

    return { medial_center, radius };
}

// 2step grid
// geometry_msgs::Point TebLocalPlannerROS::performMedialAxisClimb(
//   const geometry_msgs::Point& start_point,
//   const costmap_2d::Costmap2D& costmap,
//   const std::vector<float>& distance_field,
//   double resolution, double origin_x, double origin_y)
// {
//     unsigned int map_width = costmap.getSizeInCellsX();
//     unsigned int map_height = costmap.getSizeInCellsY();
// 
//     초기 위치
//     int cur_x = static_cast<int>((start_point.x - origin_x) / resolution);
//     int cur_y = static_cast<int>((start_point.y - origin_y) / resolution);
//     float cur_dist = distance_field[cur_y * map_width + cur_x];
// 
//     std::vector<geometry_msgs::Point> grid_search_path;
//     int max_fast_iter = 20;
//     int fast_iter = 0;
// 
//     STEP 1: step=2 hill climbing
//     int step = 2;
//     while (fast_iter < max_fast_iter)
//     {
//         bool moved = false;
//         float best_dist = cur_dist;
//         int best_x = cur_x;
//         int best_y = cur_y;
// 
//         for (int dy = -step; dy <= step; dy += step)
//         {
//             for (int dx = -step; dx <= step; dx += step)
//             {
//                 if (dx == 0 && dy == 0) continue;
// 
//                 int nx = cur_x + dx;
//                 int ny = cur_y + dy;
// 
//                 if (nx < 0 || ny < 0 || nx >= static_cast<int>(map_width) || ny >= static_cast<int>(map_height))
//                     continue;
//                 float n_dist = distance_field[ny * map_width + nx];
// 
//                 if (n_dist > best_dist)
//                 {
//                     best_dist = n_dist;
//                     best_x = nx;
//                     best_y = ny;
//                     moved = true;
//                 }
//             }
//         }
// 
//         if (!moved || std::abs(best_dist - cur_dist) < 1e-3)
//             break;
// 
//         cur_x = best_x;
//         cur_y = best_y;
//         cur_dist = best_dist;
//         fast_iter++;
//     }
// 
//     STEP 2: 1-step 주변 대칭성 (balance) 확인 및 보정 이동
//     auto compute_symmetry_score = [&](int x, int y) -> float {
//         float center = distance_field[y * map_width + x];
//         float score = 0.0;
//         int count = 0;
// 
//         for (int dy = -1; dy <= 1; ++dy) {
//             for (int dx = -1; dx <= 1; ++dx) {
//                 if (dx == 0 && dy == 0) continue;
// 
//                 int nx = x + dx;
//                 int ny = y + dy;
//                 if (nx < 0 || ny < 0 || nx >= static_cast<int>(map_width) || ny >= static_cast<int>(map_height))
//                     continue;
// 
//                 float neighbor = distance_field[ny * map_width + nx];
//                 float delta = std::abs(center - neighbor);
//                 score += std::exp(-delta);  // delta 작을수록 score 높음
//                 count++;
//             }
//         }
//         return (count > 0) ? score / count : 0.0;
//     };
// 
//     float best_symmetry = compute_symmetry_score(cur_x, cur_y);
//     int best_sym_x = cur_x;
//     int best_sym_y = cur_y;
// 
//     for (int dy = -1; dy <= 1; ++dy) {
//         for (int dx = -1; dx <= 1; ++dx) {
//             if (dx == 0 && dy == 0) continue;
// 
//             int nx = cur_x + dx;
//             int ny = cur_y + dy;
// 
//             if (nx < 0 || ny < 0 || nx >= static_cast<int>(map_width) || ny >= static_cast<int>(map_height))
//                 continue;
// 
//             float sym_score = compute_symmetry_score(nx, ny);
//             if (sym_score > best_symmetry) {
//                 best_symmetry = sym_score;
//                 best_sym_x = nx;
//                 best_sym_y = ny;
//             }
//         }
//     }
// 
//     최종 위치로 설정 (보정된 symmetry 최적 위치)
//     cur_x = best_sym_x;
//     cur_y = best_sym_y;
// 
//     geometry_msgs::Point medial_center;
//     medial_center.x = origin_x + (cur_x + 0.5) * resolution;
//     medial_center.y = origin_y + (cur_y + 0.5) * resolution;
//     medial_center.z = 0.0;
// 
//     visualization_->publishGridSearchPath(grid_search_path);
//     return medial_center;
// }

//1 step grid 
geometry_msgs::Point TebLocalPlannerROS::performMedialAxisClimb(
  const geometry_msgs::Point& start_point,
  const costmap_2d::Costmap2D& costmap,
  const std::vector<float>& distance_field,
  double resolution, double origin_x, double origin_y)
{
    unsigned int map_width = costmap.getSizeInCellsX();
    unsigned int map_height = costmap.getSizeInCellsY();

    // 초기 위치
    int cur_x = static_cast<int>((start_point.x - origin_x) / resolution);
    int cur_y = static_cast<int>((start_point.y - origin_y) / resolution);
    int cur_idx = cur_x + cur_y * map_width;
    float cur_dist = distance_field[cur_idx];

    bool moved = true;
    int max_iterations = 100;
    int iteration = 0;

    std::vector<geometry_msgs::Point> grid_search_path;
    // sample point를 medial axis 방향으로 옮기기 위한 hill-climbing 알고리즘
    while (moved && iteration < max_iterations)
    {
        moved = false;
        float best_dist = cur_dist;
        int best_x = cur_x;
        int best_y = cur_y;

        // 8방향 중 가장 distance 값이 큰 방향으로 이동
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                if (dx == 0 && dy == 0) continue;

                int nx = cur_x + dx;
                int ny = cur_y + dy;
                // 탐색 위치가 costmap 범위 내에 있는지 확인
                if (nx < 0 || ny < 0 || nx >= static_cast<int>(map_width) || ny >= static_cast<int>(map_height))
                    continue;

                int n_idx = nx + ny * map_width;
                float n_dist = distance_field[n_idx];
                // Move to the cell with a greater distance.
                if (n_dist > best_dist) {
                    best_dist = n_dist;
                    best_x = nx;
                    best_y = ny;
                    moved = true;
                }
            }
        }

        // 더 먼 셀이 없다면 종료 (local maximum)
        if (!moved)
            break;

        // // for visualization
        // geometry_msgs::Point p;
        // p.x = origin_x + (cur_x + 0.5) * resolution;
        // p.y = origin_y + (cur_y + 0.5) * resolution;
        // p.z = 0.0;
        // grid_search_path.push_back(p);


        // 이동
        cur_x = best_x;
        cur_y = best_y;
        cur_idx = cur_x + cur_y * map_width;
        cur_dist = distance_field[cur_idx];

        iteration++;
    }

    // 최종 위치를 좌표로 변환
    geometry_msgs::Point medial_center;
    medial_center.x = origin_x + (cur_x + 0.5) * resolution;
    medial_center.y = origin_y + (cur_y + 0.5) * resolution;
    medial_center.z = 0.0;

    //visualization_->publishGridSearchPath(grid_search_path);
    
    return medial_center;
}


void TebLocalPlannerROS::updateCustomViaPointsContainer(const std::vector<geometry_msgs::PoseStamped>& transformed_plan, const costmap_2d::Costmap2D& costmap)
{
    // Clear the existing via-points
    via_points_.clear();
    ROS_DEBUG("clear via point");

    // Get medial axis points
    auto medial_axis_points = detectNarrowPassages(transformed_plan, *costmap_);
    ROS_DEBUG("get medial axis points");

    // Return if no medial axis points are available
    if (medial_axis_points.empty())
        return;

    // Create a vector to store the closest plan index for each medial axis point
    std::vector<std::pair<int, Eigen::Vector2d>> indexed_medial_points;

    // Map each medial axis point to its closest transformed_plan index
    for (const auto& medial_point_pair : medial_axis_points)
    {
        const auto& medial_point = medial_point_pair.first; // geometry_msgs::Point
        Eigen::Vector2d medial_point_vec(medial_point.x, medial_point.y);

        int closest_plan_index = -1;
        double min_distance = std::numeric_limits<double>::max();

        for (size_t i = 0; i < transformed_plan.size(); ++i)
        {
            const auto& plan_pose = transformed_plan[i];
            Eigen::Vector2d plan_point_vec(plan_pose.pose.position.x, plan_pose.pose.position.y);
            double distance = (medial_point_vec - plan_point_vec).squaredNorm();

            if (distance < min_distance)
            {
                min_distance = distance;
                closest_plan_index = i;
            }
        }

        // Save the closest plan index and the medial point
        indexed_medial_points.emplace_back(closest_plan_index, medial_point_vec);
    }

    // Sort the indexed medial points by the closest plan index
    std::sort(indexed_medial_points.begin(), indexed_medial_points.end(),
              [](const std::pair<int, Eigen::Vector2d>& a, const std::pair<int, Eigen::Vector2d>& b) {
                  return a.first < b.first;
              });

    // Add sorted medial points to via_points_
    for (const auto& indexed_point : indexed_medial_points)
    {
        via_points_.emplace_back(indexed_point.second);
    }
}

void TebLocalPlannerROS::updateObstacleContainerWithCostmap()
{  
  // Add costmap obstacles if desired
  if (cfg_.obstacles.include_costmap_obstacles)
  {
    Eigen::Vector2d robot_orient = robot_pose_.orientationUnitVec();
    
    for (unsigned int i=0; i<costmap_->getSizeInCellsX()-1; ++i)
    {
      for (unsigned int j=0; j<costmap_->getSizeInCellsY()-1; ++j)
      {
        if (costmap_->getCost(i,j) == costmap_2d::LETHAL_OBSTACLE)
        {
          Eigen::Vector2d obs;
          costmap_->mapToWorld(i,j,obs.coeffRef(0), obs.coeffRef(1));
            
          // check if obstacle is interesting (e.g. not far behind the robot)
          Eigen::Vector2d obs_dir = obs-robot_pose_.position();
          if ( obs_dir.dot(robot_orient) < 0 && obs_dir.norm() > cfg_.obstacles.costmap_obstacles_behind_robot_dist  )
            continue;
            
          obstacles_.push_back(ObstaclePtr(new PointObstacle(obs)));
        }
      }
    }
  }
}

void TebLocalPlannerROS::updateObstacleContainerWithCostmapConverter()
{
  if (!costmap_converter_)
    return;
    
  //Get obstacles from costmap converter
  costmap_converter::ObstacleArrayConstPtr obstacles = costmap_converter_->getObstacles();
  if (!obstacles)
    return;

  for (std::size_t i=0; i<obstacles->obstacles.size(); ++i)
  {
    const costmap_converter::ObstacleMsg* obstacle = &obstacles->obstacles.at(i);
    const geometry_msgs::Polygon* polygon = &obstacle->polygon;

    if (polygon->points.size()==1 && obstacle->radius > 0) // Circle
    {
      obstacles_.push_back(ObstaclePtr(new CircularObstacle(polygon->points[0].x, polygon->points[0].y, obstacle->radius)));
    }
    else if (polygon->points.size()==1) // Point
    {
      obstacles_.push_back(ObstaclePtr(new PointObstacle(polygon->points[0].x, polygon->points[0].y)));
    }
    else if (polygon->points.size()==2) // Line
    {
      obstacles_.push_back(ObstaclePtr(new LineObstacle(polygon->points[0].x, polygon->points[0].y,
                                                        polygon->points[1].x, polygon->points[1].y )));
    }
    else if (polygon->points.size()>2) // Real polygon
    {
        PolygonObstacle* polyobst = new PolygonObstacle;
        for (std::size_t j=0; j<polygon->points.size(); ++j)
        {
            polyobst->pushBackVertex(polygon->points[j].x, polygon->points[j].y);
        }
        polyobst->finalizePolygon();
        obstacles_.push_back(ObstaclePtr(polyobst));
    }

    // Set velocity, if obstacle is moving
    if(!obstacles_.empty())
      obstacles_.back()->setCentroidVelocity(obstacles->obstacles[i].velocities, obstacles->obstacles[i].orientation);
  }
}


void TebLocalPlannerROS::updateObstacleContainerWithCustomObstacles()
{
  // Add custom obstacles obtained via message
  boost::mutex::scoped_lock l(custom_obst_mutex_);
  ROS_DEBUG("updateObstacleContainerWithCustomObstacles");

  if (!custom_obstacle_msg_.obstacles.empty())
  {
    ROS_DEBUG("Start to updateObstacleContainerWithCustomObstacles");
    // We only use the global header to specify the obstacle coordinate system instead of individual ones
    Eigen::Affine3d obstacle_to_map_eig;
    try 
    {
      geometry_msgs::TransformStamped obstacle_to_map =  tf_->lookupTransform(global_frame_, ros::Time(0),
                                                                              custom_obstacle_msg_.header.frame_id, ros::Time(0),
                                                                              custom_obstacle_msg_.header.frame_id, ros::Duration(cfg_.robot.transform_tolerance));
      obstacle_to_map_eig = tf2::transformToEigen(obstacle_to_map);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      obstacle_to_map_eig.setIdentity();
    }
    
    for (size_t i=0; i<custom_obstacle_msg_.obstacles.size(); ++i)
    {
      if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 1 && custom_obstacle_msg_.obstacles.at(i).radius > 0 ) // circle
      {
        Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                             custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                             custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
        obstacles_.push_back(ObstaclePtr(new CircularObstacle( (obstacle_to_map_eig * pos).head(2), custom_obstacle_msg_.obstacles.at(i).radius)));
      }
      else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 1 ) // point
      {
        Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                             custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                             custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
        obstacles_.push_back(ObstaclePtr(new PointObstacle( (obstacle_to_map_eig * pos).head(2) )));
      }
      else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 2 ) // line
      {
        Eigen::Vector3d line_start( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                                    custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                                    custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
        Eigen::Vector3d line_end( custom_obstacle_msg_.obstacles.at(i).polygon.points.back().x,
                                  custom_obstacle_msg_.obstacles.at(i).polygon.points.back().y,
                                  custom_obstacle_msg_.obstacles.at(i).polygon.points.back().z );
        obstacles_.push_back(ObstaclePtr(new LineObstacle( (obstacle_to_map_eig * line_start).head(2),
                                                           (obstacle_to_map_eig * line_end).head(2) )));
      }
      else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.empty())
      {
        ROS_WARN("Invalid custom obstacle received. List of polygon vertices is empty. Skipping...");
        continue;
      }
      else // polygon
      {
        PolygonObstacle* polyobst = new PolygonObstacle;
        for (size_t j=0; j<custom_obstacle_msg_.obstacles.at(i).polygon.points.size(); ++j)
        {
          Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points[j].x,
                               custom_obstacle_msg_.obstacles.at(i).polygon.points[j].y,
                               custom_obstacle_msg_.obstacles.at(i).polygon.points[j].z );
          polyobst->pushBackVertex( (obstacle_to_map_eig * pos).head(2) );
        }
        polyobst->finalizePolygon();
        obstacles_.push_back(ObstaclePtr(polyobst));
      }

      // Set velocity, if obstacle is moving
      if(!obstacles_.empty())
        obstacles_.back()->setCentroidVelocity(custom_obstacle_msg_.obstacles[i].velocities, custom_obstacle_msg_.obstacles[i].orientation);
    }
  }
}

void TebLocalPlannerROS::updateViaPointsContainer(const std::vector<geometry_msgs::PoseStamped>& transformed_plan, double min_separation)
{
  via_points_.clear();
  
  if (min_separation<=0)
    return;
  
  std::size_t prev_idx = 0;
  for (std::size_t i=1; i < transformed_plan.size(); ++i) // skip first one, since we do not need any point before the first min_separation [m]
  {
    // check separation to the previous via-point inserted
    if (distance_points2d( transformed_plan[prev_idx].pose.position, transformed_plan[i].pose.position ) < min_separation)
      continue;
        
    // add via-point
    via_points_.push_back( Eigen::Vector2d( transformed_plan[i].pose.position.x, transformed_plan[i].pose.position.y ) );
    prev_idx = i;
  }
  
}

Eigen::Vector2d TebLocalPlannerROS::tfPoseToEigenVector2dTransRot(const tf::Pose& tf_vel)
{
  Eigen::Vector2d vel;
  vel.coeffRef(0) = std::sqrt( tf_vel.getOrigin().getX() * tf_vel.getOrigin().getX() + tf_vel.getOrigin().getY() * tf_vel.getOrigin().getY() );
  vel.coeffRef(1) = tf::getYaw(tf_vel.getRotation());
  return vel;
}
      
      
bool TebLocalPlannerROS::pruneGlobalPlan(const tf2_ros::Buffer& tf, const geometry_msgs::PoseStamped& global_pose, std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot)
{
  ROS_DEBUG("Prune Global Plan");
  if (global_plan.empty())
    return true;
  
  try
  {
    // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
    geometry_msgs::TransformStamped global_to_plan_transform = tf.lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, ros::Time(0));
    geometry_msgs::PoseStamped robot; 
    tf2::doTransform(global_pose, robot, global_to_plan_transform);
    
    double dist_thresh_sq = dist_behind_robot*dist_behind_robot;
    
    // iterate plan until a pose close the robot is found
    std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
    std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
    while (it != global_plan.end())
    {
      double dx = robot.pose.position.x - it->pose.position.x;
      double dy = robot.pose.position.y - it->pose.position.y;
      double dist_sq = dx * dx + dy * dy;
      if (dist_sq < dist_thresh_sq)
      {
         erase_end = it;
         break;
      }
      ++it;
    }
    if (erase_end == global_plan.end())
      return false;
    
    if (erase_end != global_plan.begin())
      global_plan.erase(global_plan.begin(), erase_end);
  }
  catch (const tf::TransformException& ex)
  {
    ROS_DEBUG("Cannot prune path since no transform is available: %s\n", ex.what());
    return false;
  }
  return true;
}      

bool TebLocalPlannerROS::transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                  const geometry_msgs::PoseStamped& global_pose, const costmap_2d::Costmap2D& costmap, const std::string& global_frame, double max_plan_length,
                  std::vector<geometry_msgs::PoseStamped>& transformed_plan, int* current_goal_idx, geometry_msgs::TransformStamped* tf_plan_to_global) const
{
  // this method is a slightly modified version of base_local_planner/goal_functions.h

  const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

  transformed_plan.clear();

  try 
  {
    if (global_plan.empty())
    {
      ROS_ERROR("Received plan with zero length");
      *current_goal_idx = 0;
      return false;
    }

    // get plan_to_global_transform from plan frame to global_frame
    geometry_msgs::TransformStamped plan_to_global_transform = tf.lookupTransform(global_frame, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp,
                                                                                  plan_pose.header.frame_id, ros::Duration(cfg_.robot.transform_tolerance));

    //let's get the pose of the robot in the frame of the plan
    geometry_msgs::PoseStamped robot_pose;
    tf.transform(global_pose, robot_pose, plan_pose.header.frame_id);

    //we'll discard points on the plan that are outside the local costmap
    double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                     costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
    dist_threshold *= 0.85; // just consider 85% of the costmap size to better incorporate point obstacle that are
                           // located on the border of the local costmap
    

    int i = 0;
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist = 1e10;
    
    //we need to loop to a point on the plan that is within a certain distance of the robot
    bool robot_reached = false;
    for(int j=0; j < (int)global_plan.size(); ++j)
    {
      ROS_DEBUG("global_plan size : %d", global_plan.size());

      double x_diff = robot_pose.pose.position.x - global_plan[j].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan[j].pose.position.y;
      double new_sq_dist = x_diff * x_diff + y_diff * y_diff;

      if (robot_reached && new_sq_dist > sq_dist)
        break;

      if (new_sq_dist < sq_dist) // find closest distance
      {
        sq_dist = new_sq_dist;
        i = j;
        if (sq_dist < 0.05)      // 2.5 cm to the robot; take the immediate local minima; if it's not the global
        {
          robot_reached = true;  // minima, probably means that there's a loop in the path, and so we prefer this'
          ROS_DEBUG("find cloest pose robot_pose : (%d, %d), global_plan : (%d, %d)", robot_pose.pose.position.x, robot_pose.pose.position.y, global_plan[j].pose.position.x, global_plan[j].pose.position.y);
        }


      }
    }
    
    geometry_msgs::PoseStamped newer_pose;
    
    double plan_length = 0; // check cumulative Euclidean distance along the plan
    
    //now we'll transform until points are outside of our distance threshold
    while(i < (int)global_plan.size() && sq_dist <= sq_dist_threshold && (max_plan_length<=0 || plan_length <= max_plan_length))
    {
      const geometry_msgs::PoseStamped& pose = global_plan[i];
      tf2::doTransform(pose, newer_pose, plan_to_global_transform);

      transformed_plan.push_back(newer_pose);

      double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;
      
      // caclulate distance to previous pose
      if (i>0 && max_plan_length>0)
        plan_length += distance_points2d(global_plan[i-1].pose.position, global_plan[i].pose.position);

      ++i;
    }
        
    // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
    // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
    if (transformed_plan.empty())
    {
      tf2::doTransform(global_plan.back(), newer_pose, plan_to_global_transform);

      transformed_plan.push_back(newer_pose);
      
      // Return the index of the current goal point (inside the distance threshold)
      if (current_goal_idx) *current_goal_idx = int(global_plan.size())-1;
    }
    else
    {
      // Return the index of the current goal point (inside the distance threshold)
      if (current_goal_idx) *current_goal_idx = i-1; // subtract 1, since i was increased once before leaving the loop
    }
    
    // Return the transformation from the global plan to the global planning frame if desired
    if (tf_plan_to_global) *tf_plan_to_global = plan_to_global_transform;
  }
  catch(tf::LookupException& ex)
  {
    ROS_ERROR("No Transform available Error: %s\n", ex.what());
    return false;
  }
  catch(tf::ConnectivityException& ex) 
  {
    ROS_ERROR("Connectivity Error: %s\n", ex.what());
    return false;
  }
  catch(tf::ExtrapolationException& ex) 
  {
    ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    if (global_plan.size() > 0)
      ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

    return false;
  }
  
  ROS_DEBUG("Successfully transform global plan");
  return true;
}

double TebLocalPlannerROS::estimateLocalGoalOrientation(const std::vector<geometry_msgs::PoseStamped>& global_plan, const geometry_msgs::PoseStamped& local_goal,
              int current_goal_idx, const geometry_msgs::TransformStamped& tf_plan_to_global, int moving_average_length) const
{
  int n = (int)global_plan.size();
  
  // check if we are near the global goal already
  if (current_goal_idx > n-moving_average_length-2)
  {
    if (current_goal_idx >= n-1) // we've exactly reached the goal
    {
      return tf2::getYaw(local_goal.pose.orientation);
    }
    else
    {
      tf2::Quaternion global_orientation;
      tf2::convert(global_plan.back().pose.orientation, global_orientation);
      tf2::Quaternion rotation;
      tf2::convert(tf_plan_to_global.transform.rotation, rotation);
      // TODO(roesmann): avoid conversion to tf2::Quaternion
      return tf2::getYaw(rotation *  global_orientation);
    }     
  }
  
  // reduce number of poses taken into account if the desired number of poses is not available
  moving_average_length = std::min(moving_average_length, n-current_goal_idx-1 ); // maybe redundant, since we have checked the vicinity of the goal before
  
  std::vector<double> candidates;
  geometry_msgs::PoseStamped tf_pose_k = local_goal;
  geometry_msgs::PoseStamped tf_pose_kp1;
  
  int range_end = current_goal_idx + moving_average_length;
  for (int i = current_goal_idx; i < range_end; ++i)
  {
    // Transform pose of the global plan to the planning frame
    tf2::doTransform(global_plan.at(i+1), tf_pose_kp1, tf_plan_to_global);

    // calculate yaw angle  
    candidates.push_back( std::atan2(tf_pose_kp1.pose.position.y - tf_pose_k.pose.position.y,
        tf_pose_kp1.pose.position.x - tf_pose_k.pose.position.x ) );
    
    if (i<range_end-1) 
      tf_pose_k = tf_pose_kp1;
  }
  return average_angles(candidates);
}
      
      
void TebLocalPlannerROS::saturateVelocity(double& vx, double& vy, double& omega, double max_vel_x, double max_vel_y, double max_vel_trans, double max_vel_theta, 
              double max_vel_x_backwards) const
{
  double ratio_x = 1, ratio_omega = 1, ratio_y = 1;
  // Limit translational velocity for forward driving
  if (vx > max_vel_x)
    ratio_x = max_vel_x / vx;
  
  // limit strafing velocity
  if (vy > max_vel_y || vy < -max_vel_y)
    ratio_y = std::abs(max_vel_y / vy);
  
  // Limit angular velocity
  if (omega > max_vel_theta || omega < -max_vel_theta)
    ratio_omega = std::abs(max_vel_theta / omega);
  
  // Limit backwards velocity
  if (max_vel_x_backwards<=0)
  {
    ROS_WARN_ONCE("TebLocalPlannerROS(): Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");
  }
  else if (vx < -max_vel_x_backwards)
    ratio_x = - max_vel_x_backwards / vx;

  if (cfg_.robot.use_proportional_saturation)
  {
    double ratio = std::min(std::min(ratio_x, ratio_y), ratio_omega);
    vx *= ratio;
    vy *= ratio;
    omega *= ratio;
  }
  else
  {
    vx *= ratio_x;
    vy *= ratio_y;
    omega *= ratio_omega;
  }

  double vel_linear = std::hypot(vx, vy);
  if (vel_linear > max_vel_trans)
  {
    double max_vel_trans_ratio = max_vel_trans / vel_linear;
    vx *= max_vel_trans_ratio;
    vy *= max_vel_trans_ratio;
  }
}
     
     
double TebLocalPlannerROS::convertTransRotVelToSteeringAngle(double v, double omega, double wheelbase, double min_turning_radius) const
{
  if (omega==0 || v==0)
    return 0;
    
  double radius = v/omega;
  
  if (fabs(radius) < min_turning_radius)
    radius = double(g2o::sign(radius)) * min_turning_radius; 

  return std::atan(wheelbase / radius);
}
     

void TebLocalPlannerROS::validateFootprints(double opt_inscribed_radius, double costmap_inscribed_radius, double min_obst_dist)
{
    ROS_WARN_COND(opt_inscribed_radius + min_obst_dist < costmap_inscribed_radius,
                  "The inscribed radius of the footprint specified for TEB optimization (%f) + min_obstacle_dist (%f) are smaller "
                  "than the inscribed radius of the robot's footprint in the costmap parameters (%f, including 'footprint_padding'). "
                  "Infeasible optimziation results might occur frequently!", opt_inscribed_radius, min_obst_dist, costmap_inscribed_radius);
}
   
   
   
void TebLocalPlannerROS::configureBackupModes(std::vector<geometry_msgs::PoseStamped>& transformed_plan,  int& goal_idx)
{
    ros::Time current_time = ros::Time::now();
    
    // reduced horizon backup mode
    if (cfg_.recovery.shrink_horizon_backup && 
        goal_idx < (int)transformed_plan.size()-1 && // we do not reduce if the goal is already selected (because the orientation might change -> can introduce oscillations)
       (no_infeasible_plans_>0 || (current_time - time_last_infeasible_plan_).toSec() < cfg_.recovery.shrink_horizon_min_duration )) // keep short horizon for at least a few seconds
    {
        ROS_INFO_COND(no_infeasible_plans_==1, "Activating reduced horizon backup mode for at least %.2f sec (infeasible trajectory detected).", cfg_.recovery.shrink_horizon_min_duration);


        // Shorten horizon if requested
        // reduce to 50 percent:
        int horizon_reduction = goal_idx/2;
        
        if (no_infeasible_plans_ > 9)
        {
            ROS_INFO_COND(no_infeasible_plans_==10, "Infeasible trajectory detected 10 times in a row: further reducing horizon...");
            horizon_reduction /= 2;
        }
        
        // we have a small overhead here, since we already transformed 50% more of the trajectory.
        // But that's ok for now, since we do not need to make transformGlobalPlan more complex 
        // and a reduced horizon should occur just rarely.
        int new_goal_idx_transformed_plan = int(transformed_plan.size()) - horizon_reduction - 1;
        goal_idx -= horizon_reduction;
        if (new_goal_idx_transformed_plan>0 && goal_idx >= 0)
            transformed_plan.erase(transformed_plan.begin()+new_goal_idx_transformed_plan, transformed_plan.end());
        else
            goal_idx += horizon_reduction; // this should not happen, but safety first ;-) 
    }
    
    
    // detect and resolve oscillations
    if (cfg_.recovery.oscillation_recovery)
    {
        double max_vel_theta;
        double max_vel_current = last_cmd_.linear.x >= 0 ? cfg_.robot.max_vel_x : cfg_.robot.max_vel_x_backwards;
        if (cfg_.robot.min_turning_radius!=0 && max_vel_current>0)
            max_vel_theta = std::max( max_vel_current/std::abs(cfg_.robot.min_turning_radius),  cfg_.robot.max_vel_theta );
        else
            max_vel_theta = cfg_.robot.max_vel_theta;
        
        failure_detector_.update(last_cmd_, cfg_.robot.max_vel_x, cfg_.robot.max_vel_x_backwards, max_vel_theta,
                               cfg_.recovery.oscillation_v_eps, cfg_.recovery.oscillation_omega_eps);
        
        bool oscillating = failure_detector_.isOscillating();
        bool recently_oscillated = (ros::Time::now()-time_last_oscillation_).toSec() < cfg_.recovery.oscillation_recovery_min_duration; // check if we have already detected an oscillation recently
        
        if (oscillating)
        {
            if (!recently_oscillated)
            {
                // save current turning direction
                if (robot_vel_.angular.z > 0)
                    last_preferred_rotdir_ = RotType::left;
                else
                    last_preferred_rotdir_ = RotType::right;
                ROS_WARN("TebLocalPlannerROS: possible oscillation (of the robot or its local plan) detected. Activating recovery strategy (prefer current turning direction during optimization).");
            }
            time_last_oscillation_ = ros::Time::now();  
            planner_->setPreferredTurningDir(last_preferred_rotdir_);
        }
        else if (!recently_oscillated && last_preferred_rotdir_ != RotType::none) // clear recovery behavior
        {
            last_preferred_rotdir_ = RotType::none;
            planner_->setPreferredTurningDir(last_preferred_rotdir_);
            ROS_INFO("TebLocalPlannerROS: oscillation recovery disabled/expired.");
        }
    }

}
     
void TebLocalPlannerROS::customObstacleCB(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg)
{
  boost::mutex::scoped_lock l(custom_obst_mutex_);
  custom_obstacle_msg_ = *obst_msg;  
}

void TebLocalPlannerROS::customViaPointsCB(const nav_msgs::Path::ConstPtr& via_points_msg)
{
  ROS_INFO_ONCE("Via-points received. This message is printed once.");

  if (cfg_.trajectory.global_plan_viapoint_sep > 0)
  {
    ROS_WARN("Via-points are already obtained from the global plan (global_plan_viapoint_sep>0)."
             "Ignoring custom via-points.");
    custom_via_points_active_ = false;
    return;
  }

  boost::mutex::scoped_lock l(via_point_mutex_);
  via_points_.clear();
  for (const geometry_msgs::PoseStamped& pose : via_points_msg->poses)
  {
    via_points_.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }

  custom_via_points_active_ = !via_points_.empty();
}
     
RobotFootprintModelPtr TebLocalPlannerROS::getRobotFootprintFromParamServer(const ros::NodeHandle& nh, const TebConfig& config)
{
  std::string model_name; 
  if (!nh.getParam("footprint_model/type", model_name))
  {
    ROS_INFO("No robot footprint model specified for trajectory optimization. Using point-shaped model.");
    return boost::make_shared<PointRobotFootprint>();
  }
    
  // point  
  if (model_name.compare("point") == 0)
  {
    ROS_INFO("Footprint model 'point' loaded for trajectory optimization.");
    return boost::make_shared<PointRobotFootprint>(config.obstacles.min_obstacle_dist);
  }
  
  // circular
  if (model_name.compare("circular") == 0)
  {
    // get radius
    double radius;
    if (!nh.getParam("footprint_model/radius", radius))
    {
      ROS_ERROR_STREAM("Footprint model 'circular' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace() 
                       << "/footprint_model/radius' does not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    ROS_INFO_STREAM("Footprint model 'circular' (radius: " << radius <<"m) loaded for trajectory optimization.");
    return boost::make_shared<CircularRobotFootprint>(radius);
  }
  
  // line
  if (model_name.compare("line") == 0)
  {
    // check parameters
    if (!nh.hasParam("footprint_model/line_start") || !nh.hasParam("footprint_model/line_end"))
    {
      ROS_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace() 
                       << "/footprint_model/line_start' and/or '.../line_end' do not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    // get line coordinates
    std::vector<double> line_start, line_end;
    nh.getParam("footprint_model/line_start", line_start);
    nh.getParam("footprint_model/line_end", line_end);
    if (line_start.size() != 2 || line_end.size() != 2)
    {
      ROS_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace() 
                       << "/footprint_model/line_start' and/or '.../line_end' do not contain x and y coordinates (2D). Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    
    ROS_INFO_STREAM("Footprint model 'line' (line_start: [" << line_start[0] << "," << line_start[1] <<"]m, line_end: ["
                     << line_end[0] << "," << line_end[1] << "]m) loaded for trajectory optimization.");
    return boost::make_shared<LineRobotFootprint>(Eigen::Map<const Eigen::Vector2d>(line_start.data()), Eigen::Map<const Eigen::Vector2d>(line_end.data()), config.obstacles.min_obstacle_dist);
  }
  
  // two circles
  if (model_name.compare("two_circles") == 0)
  {
    // check parameters
    if (!nh.hasParam("footprint_model/front_offset") || !nh.hasParam("footprint_model/front_radius") 
        || !nh.hasParam("footprint_model/rear_offset") || !nh.hasParam("footprint_model/rear_radius"))
    {
      ROS_ERROR_STREAM("Footprint model 'two_circles' cannot be loaded for trajectory optimization, since params '" << nh.getNamespace()
                       << "/footprint_model/front_offset', '.../front_radius', '.../rear_offset' and '.../rear_radius' do not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    double front_offset, front_radius, rear_offset, rear_radius;
    nh.getParam("footprint_model/front_offset", front_offset);
    nh.getParam("footprint_model/front_radius", front_radius);
    nh.getParam("footprint_model/rear_offset", rear_offset);
    nh.getParam("footprint_model/rear_radius", rear_radius);
    ROS_INFO_STREAM("Footprint model 'two_circles' (front_offset: " << front_offset <<"m, front_radius: " << front_radius 
                    << "m, rear_offset: " << rear_offset << "m, rear_radius: " << rear_radius << "m) loaded for trajectory optimization.");
    return boost::make_shared<TwoCirclesRobotFootprint>(front_offset, front_radius, rear_offset, rear_radius);
  }

  // polygon
  if (model_name.compare("polygon") == 0)
  {

    // check parameters
    XmlRpc::XmlRpcValue footprint_xmlrpc;
    if (!nh.getParam("footprint_model/vertices", footprint_xmlrpc) )
    {
      ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace() 
                       << "/footprint_model/vertices' does not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    // get vertices
    if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      try
      {
        Point2dContainer polygon = makeFootprintFromXMLRPC(footprint_xmlrpc, "/footprint_model/vertices");
        ROS_INFO_STREAM("Footprint model 'polygon' loaded for trajectory optimization.");
        return boost::make_shared<PolygonRobotFootprint>(polygon);
      } 
      catch(const std::exception& ex)
      {
        ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization: " << ex.what() << ". Using point-model instead.");
        return boost::make_shared<PointRobotFootprint>();
      }
    }
    else
    {
      ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace() 
                       << "/footprint_model/vertices' does not define an array of coordinates. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    
  }
  
  // otherwise
  ROS_WARN_STREAM("Unknown robot footprint model specified with parameter '" << nh.getNamespace() << "/footprint_model/type'. Using point model instead.");
  return boost::make_shared<PointRobotFootprint>();
}
         
       
       
       
Point2dContainer TebLocalPlannerROS::makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue& footprint_xmlrpc, const std::string& full_param_name)
{
   // Make sure we have an array of at least 3 elements.
   if (footprint_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray ||
       footprint_xmlrpc.size() < 3)
   {
     ROS_FATAL("The footprint must be specified as list of lists on the parameter server, %s was specified as %s",
                full_param_name.c_str(), std::string(footprint_xmlrpc).c_str());
     throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least "
                              "3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
   }
 
   Point2dContainer footprint;
   Eigen::Vector2d pt;
 
   for (int i = 0; i < footprint_xmlrpc.size(); ++i)
   {
     // Make sure each element of the list is an array of size 2. (x and y coordinates)
     XmlRpc::XmlRpcValue point = footprint_xmlrpc[ i ];
     if (point.getType() != XmlRpc::XmlRpcValue::TypeArray ||
         point.size() != 2)
     {
       ROS_FATAL("The footprint (parameter %s) must be specified as list of lists on the parameter server eg: "
                 "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.",
                  full_param_name.c_str());
       throw std::runtime_error("The footprint must be specified as list of lists on the parameter server eg: "
                               "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
    }

    pt.x() = getNumberFromXMLRPC(point[ 0 ], full_param_name);
    pt.y() = getNumberFromXMLRPC(point[ 1 ], full_param_name);

    footprint.push_back(pt);
  }
  return footprint;
}

double TebLocalPlannerROS::getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& full_param_name)
{
  // Make sure that the value we're looking at is either a double or an int.
  if (value.getType() != XmlRpc::XmlRpcValue::TypeInt &&
      value.getType() != XmlRpc::XmlRpcValue::TypeDouble)
  {
    std::string& value_string = value;
    ROS_FATAL("Values in the footprint specification (param %s) must be numbers. Found value %s.",
               full_param_name.c_str(), value_string.c_str());
     throw std::runtime_error("Values in the footprint specification must be numbers");
   }
   return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
}

} // end namespace teb_local_planner


