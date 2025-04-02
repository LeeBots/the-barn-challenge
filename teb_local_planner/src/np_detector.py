import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
import numpy as np

class NarrowPassageDetector:
    def __init__(self, threshold, delta_convergence, iteration_limit, costmap, num_samples, marker_pub):
        self.threshold = threshold
        self.delta_convergence = delta_convergence
        self.iteration_limit = iteration_limit
        self.costmap = costmap
        self.num_samples = num_samples
        self.marker_pub = marker_pub

    def detect_narrow_passages(self):
        narrow_passages = []
        samples = self.generate_samples()

        for sample in samples:
            center = np.array([sample.x, sample.y])
            radius = self.threshold

            for iteration in range(self.iteration_limit):
                if self.is_obstacle_in_radius(center, radius):
                    break
                radius -= self.delta_convergence

                if radius <= 0:
                    rospy.logwarn("Radius reached zero before finding a medial ball.")
                    break

            if radius > 0 and radius < self.threshold:
                narrow_passages.append((Point(center[0], center[1], 0.0), radius))

            self.visualize_medial_ball(Point(center[0], center[1], 0.0), radius)

        return narrow_passages

    def generate_samples(self):
        samples = []
        origin_x = self.costmap.info.origin.position.x
        origin_y = self.costmap.info.origin.position.y
        resolution = self.costmap.info.resolution
        width = self.costmap.info.width
        height = self.costmap.info.height

        while len(samples) < self.num_samples:
            mx = np.random.randint(0, width)
            my = np.random.randint(0, height)
            index = my * width + mx

            if self.costmap.data[index] == 0:  # FREE_SPACE only
                sample = Point()
                sample.x = origin_x + mx * resolution
                sample.y = origin_y + my * resolution
                sample.z = 0.0
                samples.append(sample)

        return samples

    def is_obstacle_in_radius(self, center, radius):
        origin_x = self.costmap.info.origin.position.x
        origin_y = self.costmap.info.origin.position.y
        resolution = self.costmap.info.resolution
        width = self.costmap.info.width
        height = self.costmap.info.height

        min_x = max(0, int((center[0] - radius - origin_x) / resolution))
        max_x = min(width - 1, int((center[0] + radius - origin_x) / resolution))
        min_y = max(0, int((center[1] - radius - origin_y) / resolution))
        max_y = min(height - 1, int((center[1] + radius - origin_y) / resolution))

        for my in range(min_y, max_y + 1):
            for mx in range(min_x, max_x + 1):
                index = my * width + mx
                if self.costmap.data[index] != 0:  # Obstacle detected
                    distance = np.linalg.norm([
                        origin_x + mx * resolution - center[0],
                        origin_y + my * resolution - center[1]
                    ])
                    if distance <= radius:
                        return True
        return False

    def visualize_medial_ball(self, center, radius):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "medial_balls"
        marker.id = int(center.x * 1000 + center.y * 1000)
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.02
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        for angle in np.linspace(0, 2 * np.pi, 36):
            p = Point()
            p.x = center.x + radius * np.cos(angle)
            p.y = center.y + radius * np.sin(angle)
            p.z = 0.0
            marker.points.append(p)

        self.marker_pub.publish(marker)

        center_marker = Marker()
        center_marker.header.frame_id = "map"
        center_marker.header.stamp = rospy.Time.now()
        center_marker.ns = "medial_ball_centers"
        center_marker.id = int(center.x * 1000 + center.y * 1000 + 1)
        center_marker.type = Marker.SPHERE
        center_marker.action = Marker.ADD

        center_marker.scale.x = 0.1
        center_marker.scale.y = 0.1
        center_marker.scale.z = 0.1
        center_marker.color.a = 1.0
        center_marker.color.r = 0.0
        center_marker.color.g = 1.0
        center_marker.color.b = 0.0

        center_marker.pose.position = center

        self.marker_pub.publish(center_marker)

latest_costmap_msg = None

def costmap_callback(msg):
    global latest_costmap_msg
    latest_costmap_msg = msg
    rospy.loginfo_throttle(5, "Costmap data updated.")

def main():
    rospy.init_node("narrow_passage_detector")

    threshold = rospy.get_param("~threshold", 0.7)
    delta_convergence = rospy.get_param("~delta_convergence", 0.01)
    iteration_limit = rospy.get_param("~iteration_limit", 100)
    num_samples = rospy.get_param("~num_samples", 10)

    rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, costmap_callback)
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=5)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if latest_costmap_msg:
            detector = NarrowPassageDetector(threshold, delta_convergence, iteration_limit, latest_costmap_msg, num_samples, marker_pub)
            narrow_passages = detector.detect_narrow_passages()

            for passage in narrow_passages:
                rospy.loginfo("Narrow passage detected at (x=%.2f, y=%.2f) with medial radius %.2f",
                              passage[0].x, passage[0].y, passage[1])
        else:
            rospy.logwarn_throttle(5, "Waiting for costmap data...")

        rate.sleep()

if __name__ == "__main__":
    main()