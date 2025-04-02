#include "ros/ros.h"
#include <teb_local_planner/nanoflann.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <random>
#include <cmath>
#include <ros/ros.h>

// base local planner base class and utilities
#include <nav_core/base_local_planner.h>
#include <mbf_costmap_core/costmap_controller.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/costmap_model.h>


// timed-elastic-band related classes
#include <teb_local_planner/optimal_planner.h>
#include <teb_local_planner/homotopy_class_planner.h>
#include <teb_local_planner/visualization.h>
#include <teb_local_planner/recovery_behaviors.h>

// message types
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <costmap_converter/ObstacleMsg.h>

// transforms
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>

// costmap
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_converter/costmap_converter_interface.h>


// dynamic reconfigure
#include <teb_local_planner/TebLocalPlannerReconfigureConfig.h>
#include <dynamic_reconfigure/server.h>

// boost classes
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

class NarrowPassageDetector
{
public:
    NarrowPassageDetector(double thre, const nav_msgs::OccupancyGrid::ConstPtr& local_costmap, const nav_msgs::OccupancyGrid::ConstPtr& global_costmap, unsigned int num_samples, const nav_msgs::Path::ConstPtr& global_plan)
        : thre_(thre), local_costmap_(local_costmap), global_costmap_(global_costmap), num_samples_(num_samples), global_plan_(global_plan)
    {
    }

    std::vector<std::pair<geometry_msgs::Point, double>> detectNarrowPassages()
    {
        std::vector<std::pair<geometry_msgs::Point, double>> medial_axis_point;

        std::vector<geometry_msgs::Point> samples = generateSamples();
        // visualization_->visualizeSamples(samples);

        double obst_radius = 0.5;

        std::vector<geometry_msgs::Point> obstacle_points; // Store samples with obstacles

        for (const auto& sample : samples)
        {
            auto obstacles_in_circle = getObstaclePointsInCircle(sample, obst_radius);

            if (obstacles_in_circle)
            {
                // visualization_->visualizeNarrowSpace(sample, obst_radius);
                obstacle_points.push_back(sample);
            }
        }

        // 장애물이 포함된 샘플로 Medial Ball 생성
        for (const auto& point : obstacle_points)
        {
            //auto medial_ball = findMedialBallWithCostmapKDTree(point, *costmap_);
            double medial_radius = findMedialBallRadius(point).second;
            geometry_msgs::Point final_center = findMedialBallRadius(point).first;

            // threshold 이상인 경우 추가하지 않음
            if (medial_radius < thre_ && medial_radius >= 0.25)
            {
                medial_axis_point.emplace_back(final_center, medial_radius);
                // visualization_->visualizeMedialBall(final_center, medial_radius);
            }
        }

        return medial_axis_point;
    }

private:
    double thre_;
    nav_msgs::OccupancyGrid::ConstPtr local_costmap_;
    nav_msgs::OccupancyGrid::ConstPtr global_costmap_;
    nav_msgs::Path::ConstPtr global_plan_;
    unsigned int num_samples_;

    double euclideanDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
    {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    }

    std::vector<geometry_msgs::Point> generateSamples()
    {   
        std::vector<geometry_msgs::Point> samples;

        double origin_x = global_costmap_->info.origin.position.x;
        double origin_y = global_costmap_->info.origin.position.y;
        double resolution = global_costmap_->info.resolution;
        unsigned int width = global_costmap_->info.width;
        unsigned int height = global_costmap_->info.height;

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis_x(0, width - 1);
        std::uniform_int_distribution<> dis_y(0, height - 1);

        // Parameters for the sampling bias
        double rho = 0.8;         // Expansion coefficient
        double lambda = 1.0;      // Decay rate for exponential bias
        double threshold_distance = 0.5;

        while (samples.size() < num_samples_)
        {
            int mx = dis_x(gen);
            int my = dis_y(gen);
            int index = my * width + mx;

            if (global_costmap_->data[index] == 0) // FREE_SPACE only
            {
                geometry_msgs::Point sample;
                sample.x = origin_x + mx * resolution;
                sample.y = origin_y + my * resolution;

                // **Check if the sample is within a threshold distance to the global path**
                bool within_threshold = false;
                for (const auto& pose : global_plan_->poses)
                {
                    double d_v = euclideanDistance(sample, pose.pose.position);
                    if (d_v <= threshold_distance)
                    {
                        within_threshold = true;
                        break;  // No need to check further if already within threshold
                    }
                }

                // If sample is within threshold distance from any point in the global path, accept it
                if (within_threshold)
                {
                    // Apply exponential bias (optional, if you want to prioritize sampling near the path)
                    double d_G = 0.0;  // You could calculate this distance as needed for bias
                    double sampling_weight = lambda * std::exp(-lambda * d_G);

                    if (std::uniform_real_distribution<>(0, 1)(gen) < sampling_weight)
                    {
                        samples.push_back(sample);
                    }
                }
            }
        }

        return samples;
    }

    double calculateAngle(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, const geometry_msgs::Point& center)
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

            // 클램프를 사용하지 않고 범위를 제한
            if (cosine_angle < -1.0)
                cosine_angle = -1.0;
            else if (cosine_angle > 1.0)
                cosine_angle = 1.0;

            return std::acos(cosine_angle); // 각도 (라디안)
        }
        return 0.0;
    }

    bool getObstaclePointsInCircle(const geometry_msgs::Point& center, double radius)
    {
        std::vector<geometry_msgs::Point> grid_obstacle_points;
        int min_obstacles = 5; // 최소 장애물 점 개수

        for (double angle = 0; angle < 2 * M_PI; angle += M_PI / 36)
        {
            double r_x = center.x + radius * std::cos(angle);
            double r_y = center.y + radius * std::sin(angle);

            if (isObstacleOrUnknown(r_x, r_y))
            {
                geometry_msgs::Point o_point;
                o_point.x = r_x;
                o_point.y = r_y;
                o_point.z = 0.0;
                grid_obstacle_points.push_back(o_point);
            }
        }

        ROS_INFO("Number of obstacle points detected: %lu", grid_obstacle_points.size());

        // 최소 개수 이상의 장애물 점이 있어야 유효한 장애물로 간주
        if (grid_obstacle_points.size() < min_obstacles)
        {
            return false;
        }

        return true;
    }

    // Find Medial ball using KD-Tree

    // struct ObstacleData
    // {
    //     std::vector<std::pair<double, double>> obstacle_points;

    //     // KDTree 인터페이스
    //     inline size_t kdtree_get_point_count() const { return obstacle_points.size(); }

    //     inline double kdtree_get_pt(const size_t idx, const size_t dim) const
    //     {
    //         return dim == 0 ? obstacle_points[idx].first : obstacle_points[idx].second;
    //     }

    //     template <class BBOX>
    //     bool kdtree_get_bbox(BBOX&) const { return false; }
    // };

    // ObstacleData extractObstaclesFromCostmap(const nav_msgs::OccupancyGrid& costmap)
    // {
    //     ObstacleData data;
    //     const auto& map = costmap.data;
    //     const double resolution = costmap.info.resolution;
    //     const double origin_x = costmap.info.origin.position.x;
    //     const double origin_y = costmap.info.origin.position.y;

    //     // Costmap 데이터 순회
    //     for (size_t i = 0; i < costmap.info.height; ++i)
    //     {
    //         for (size_t j = 0; j < costmap.info.width; ++j)
    //         {
    //             size_t idx = i * costmap.info.width + j;
    //             if (map[idx] > 50) // 장애물로 간주 (threshold 조정 가능)
    //             {
    //                 double x = origin_x + j * resolution;
    //                 double y = origin_y + i * resolution;
    //                 data.obstacle_points.emplace_back(x, y);
    //             }
    //         }
    //     }
    //     return data;
    // }
    
    // using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
    //     nanoflann::L2_Simple_Adaptor<double, ObstacleData>,
    //     ObstacleData,
    //     2>;

    // std::pair<geometry_msgs::Point, double> findMedialBallWithCostmapKDTree(const geometry_msgs::Point& point, const nav_msgs::OccupancyGrid& costmap)
    // {
    //     // Costmap에서 장애물 데이터를 추출하고 KD Tree 초기화
    //     ObstacleData obstacles = extractObstaclesFromCostmap(costmap);
    //     KDTree kd_tree(2, obstacles, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    //     kd_tree.buildIndex();

    //     double max_radius = std::min(costmap.info.width, costmap.info.height) * costmap.info.resolution / 2.0;
    //     double step_size = 0.01;
    //     double radius = 0.2;
    //     geometry_msgs::Point center = point;

    //     while (radius <= max_radius)
    //     {
    //         std::vector<nanoflann::ResultItem<unsigned int, double>> result_items;

    //         // KD Tree를 사용하여 반지름 내 장애물 점 검색
    //         nanoflann::SearchParameters params;
    //         const double search_radius = radius * radius; // 거리 제곱으로 검색
    //         size_t found_count = kd_tree.radiusSearch(&center.x, search_radius, result_items, params);

    //         if (found_count == 0)
    //         {
    //             // 경계 점이 발견되지 않으면 반지름 증가
    //             radius += step_size;
    //             continue;
    //         }

    //         // 중심 이동 계산
    //         geometry_msgs::Point mean_boundary_point;
    //         for (const auto& item : result_items)
    //         {
    //             unsigned int idx = static_cast<unsigned int>(item.first);
    //             mean_boundary_point.x += obstacles.obstacle_points[idx].first;
    //             mean_boundary_point.y += obstacles.obstacle_points[idx].second;
    //         }
    //         mean_boundary_point.x /= found_count;
    //         mean_boundary_point.y /= found_count;

    //         double vector_x = mean_boundary_point.x - center.x;
    //         double vector_y = mean_boundary_point.y - center.y;

    //         double magnitude = std::sqrt(vector_x * vector_x + vector_y * vector_y);
    //         if (magnitude > 0)
    //         {
    //             center.x -= (vector_x / magnitude) * step_size;
    //             center.y -= (vector_y / magnitude) * step_size;
    //         }

    //         // 종료 조건 체크
    //         if (found_count > 1)
    //         {
    //             // 마지막 경계 점 변환
    //             geometry_msgs::Point last_boundary_point;
    //             unsigned int last_idx = static_cast<unsigned int>(result_items.back().first);
    //             last_boundary_point.x = obstacles.obstacle_points[last_idx].first;
    //             last_boundary_point.y = obstacles.obstacle_points[last_idx].second;
    //             last_boundary_point.z = 0.0; // z 값은 0으로 설정

    //             double angle = calculateAngle(mean_boundary_point, last_boundary_point, center);
    //             if (angle > M_PI / 2.0)
    //             {
    //                 return {center, radius};
    //             }
    //         }

    //         // 반지름 증가
    //         radius += step_size;
    //     }

    //     return {center, radius};
    // }

    std::pair<geometry_msgs::Point, double> findMedialBallRadius(const geometry_msgs::Point& point)
    {
        double max_radius = std::min(local_costmap_->info.width, local_costmap_->info.height) * local_costmap_->info.resolution / 2.0;
        double step_size = 0.01;
        double radius = 0.2;
        geometry_msgs::Point center = point;

        while (radius <= max_radius)
        {
            std::vector<geometry_msgs::Point> boundary_points;

            // 원의 경계 점 계산
            for (double angle = 0; angle < 2 * M_PI; angle += M_PI / 36)
            {
                double sample_x = center.x + radius * std::cos(angle);
                double sample_y = center.y + radius * std::sin(angle);

                if (isObstacleOrUnknown(sample_x, sample_y))
                {
                    geometry_msgs::Point boundary_point;
                    boundary_point.x = sample_x;
                    boundary_point.y = sample_y;
                    boundary_point.z = 0.0; // 기본값 설정
                    boundary_points.push_back(boundary_point);
                }
            }

            // 경계 점이 발견되지 않으면 반지름 증가
            if (boundary_points.empty())
            {
                radius += step_size;
                continue;
            }

            // 경계 점이 두 개 이상일 경우 종료 조건
            if (boundary_points.size() > 1)
            {
                const auto& last_boundary_point = boundary_points.back();
                double angle = calculateAngle(boundary_points.front(), last_boundary_point, center);

                if (angle > M_PI / 2.0)
                {
                    return {center, radius}; 
                }
                
            }

            // 중심 이동 계산
            const auto& boundary_point = boundary_points.front();

            double vector_x = boundary_point.x - center.x;
            double vector_y = boundary_point.y - center.y;

            double magnitude = std::sqrt(vector_x * vector_x + vector_y * vector_y);
            if (magnitude > 0)
            {
                center.x -= (vector_x / magnitude) * step_size;
                center.y -= (vector_y / magnitude) * step_size;
            }

            // 반지름 증가
            radius += step_size;

        }

        return {center, radius}; // 최대 반지름에 도달한 경우 반환
    }

    bool isObstacleOrUnknown(double x, double y)
    {
        int mx = static_cast<int>((x - global_costmap_->info.origin.position.x) / global_costmap_->info.resolution);
        int my = static_cast<int>((y - global_costmap_->info.origin.position.y) / global_costmap_->info.resolution);

        if (mx < 0 || my < 0 || mx >= static_cast<int>(global_costmap_->info.width) || my >= static_cast<int>(global_costmap_->info.height))
        {
            return true; // Out of bounds
        }

        int index = my * global_costmap_->info.width + mx;
        return global_costmap_->data[index] >= 100 || global_costmap_->data[index] == -1; // LETHAL_OBSTACLE or NO_INFORMATION
    }

    // void visualizeSamples(const std::vector<geometry_msgs::Point>& samples)
    // {
    //     visualization_msgs::Marker marker;
    //     marker.header.frame_id = "map";
    //     marker.header.stamp = ros::Time::now();
    //     marker.ns = "samples";
    //     marker.id = 0;
    //     marker.type = visualization_msgs::Marker::POINTS;
    //     marker.action = visualization_msgs::Marker::ADD;

    //     marker.scale.x = 0.1;
    //     marker.scale.y = 0.1;
    //     marker.color.a = 1.0;
    //     marker.color.r = 0.0;
    //     marker.color.g = 1.0;
    //     marker.color.b = 0.0;

    //     for (const auto& sample : samples)
    //     {
    //         geometry_msgs::Point p;
    //         p.x = sample.x;
    //         p.y = sample.y;
    //         p.z = 0.0;
    //         marker.points.push_back(p);
    //     }

    //     marker_pub_.publish(marker);
    // }

    // void visualizeMedialBall(const geometry_msgs::Point& center, double radius)
    // {
    //     visualization_msgs::Marker marker;
    //     marker.header.frame_id = "map";
    //     marker.header.stamp = ros::Time::now();
    //     marker.ns = "medial_balls";
    //     marker.id = static_cast<int>(center.x * 1000 + center.y * 1000);
    //     marker.type = visualization_msgs::Marker::LINE_STRIP;
    //     marker.action = visualization_msgs::Marker::ADD;

    //     marker.scale.x = 0.02;
    //     marker.color.a = 1.0;
    //     marker.color.r = 1.0;
    //     marker.color.g = 0.0;
    //     marker.color.b = 0.0;

    //     // Draw the medial ball centered at the new position
    //     for (double angle = 0; angle <= 2 * M_PI; angle += M_PI / 36)
    //     {
    //         geometry_msgs::Point p;
    //         p.x = center.x + radius * std::cos(angle);
    //         p.y = center.y + radius * std::sin(angle);
    //         p.z = 0.0;
    //         marker.points.push_back(p);
    //     }

    //     ros::Duration lifetime(1.0);
    //     marker.lifetime = lifetime;

    //     marker_pub_.publish(marker);
    // }

    // void visualizeNarrowSpace(const geometry_msgs::Point& center, double radius)
    // {
    //     visualization_msgs::Marker marker;
    //     marker.header.frame_id = "map"; // Replace "map" with the appropriate frame if needed
    //     marker.header.stamp = ros::Time::now();
    //     marker.ns = "narrow_space";
    //     marker.id = static_cast<int>(center.x * 1000 + center.y * 1000); // Unique ID for each marker
    //     marker.type = visualization_msgs::Marker::CYLINDER; // Use CYLINDER to represent the narrow space
    //     marker.action = visualization_msgs::Marker::ADD;

    //     // Set the position of the marker
    //     marker.pose.position.x = center.x;
    //     marker.pose.position.y = center.y;
    //     marker.pose.position.z = 0.0; // Adjust the height as needed
    //     marker.pose.orientation.x = 0.0;
    //     marker.pose.orientation.y = 0.0;
    //     marker.pose.orientation.z = 0.0;
    //     marker.pose.orientation.w = 1.0;

    //     // Set the scale (diameter = 2 * radius)
    //     marker.scale.x = 2 * radius;
    //     marker.scale.y = 2 * radius;
    //     marker.scale.z = 0.1; // Height of the cylinder (can be adjusted as needed)

    //     // Set the color (e.g., red for narrow spaces)
    //     marker.color.a = 0.2; // Transparency
    //     marker.color.r = 0.0;
    //     marker.color.g = 1.0;
    //     marker.color.b = 0.0;

    //     // Set lifetime (optional, so it doesn't persist forever)
    //     marker.lifetime = ros::Duration(1.0); // Adjust the duration as needed

    //     // Publish the marker
    //     marker_pub_.publish(marker);
    // }
};

// nav_msgs::OccupancyGrid::ConstPtr latest_local_costmap_msg;
// nav_msgs::OccupancyGrid::ConstPtr latest_global_costmap_msg;
// nav_msgs::Path::ConstPtr global_plan_msg;

// void localcostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
// {
//     latest_local_costmap_msg = msg;
//     ROS_INFO_THROTTLE(5, "Costmap data updated.");
// }
// void globalcostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
// {
//     latest_global_costmap_msg = msg;
//     ROS_INFO_THROTTLE(5, "Costmap data updated.");
// }

// void globalPlanCallback(const nav_msgs::Path::ConstPtr& msg)
// {
//     global_plan_msg = msg;
//     ROS_INFO_THROTTLE(5, "Global path updated.");
// }

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "medial_axis_detector");
//     ros::NodeHandle nh;

//     ros::Subscriber local_costmap_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/move_base/local_costmap/costmap", 5, localcostmapCallback);
//     ros::Subscriber gobal_costmap_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/move_base/global_costmap/costmap", 5, globalcostmapCallback);
//     ros::Subscriber global_plan = nh.subscribe<nav_msgs::Path>("/move_base/TebLocalPlannerROS/global_plan", 5, globalPlanCallback);
    
//     ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 5);

//     ros::Rate rate(10);
//     while (ros::ok())
//     {
//         if (latest_local_costmap_msg && latest_global_costmap_msg && global_plan_msg)
//         {
//             NarrowPassageDetector detector(threshold, latest_local_costmap_msg, latest_global_costmap_msg, num_samples, marker_pub, global_plan_msg);
//             std::vector<std::pair<geometry_msgs::Point, double>> medial_axis_point = detector.detectNarrowPassages();

//             for (const auto& point : medial_axis_point)
//             {
//                 ROS_INFO("Narrow passage detected at (x=%.2f, y=%.2f) with medial radius %.2f",
//                          point.first.x, point.first.y, point.second);
//             }
//         }
//         else
//         {
//             ROS_WARN_THROTTLE(5, "Waiting for costmap and global plan data...");
//         }

//         ros::spinOnce();
//         rate.sleep();
//     }

//     return 0;
// }