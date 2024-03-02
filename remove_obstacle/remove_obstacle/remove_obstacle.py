#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np

class PointCloudFilter(Node):
    def __init__(self):
        super().__init__('point_cloud_filter')
        
        self.sub = self.create_subscription(PointCloud2, '/scanner/cloud', self.point_cloud_callback, 10)
        
        self.pub = self.create_publisher(PointCloud2, '/scanner/cloud_remove_obstacle', 10)

    def point_cloud_callback(self, msg):
        
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        pointcloud_data = np.array(list(points), dtype=np.float32)

        distance_to_center = np.sqrt(pointcloud_data[:, 0]**2 + pointcloud_data[:, 1]**2 + pointcloud_data[:, 2]**2)

        filtered_pointcloud_data = pointcloud_data[distance_to_center > 0.3]

        filtered_msg = pc2.create_cloud_xyz32(header=msg.header, points=filtered_pointcloud_data)

        self.publisher.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    pcl_filter = PointCloudFilter()
    rclpy.spin(pcl_filter)
    pcl_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
