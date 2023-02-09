#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped
from std_srvs.srv import Empty
import cs_tools as cs
import math
import csv
import os
from ament_index_python.packages import get_package_share_directory

def csv_save_point(filename, data):
    with open(filename, 'w') as csvfile:
        field = ['time', 'pos_x', 'pos_y']
        writer = csv.DictWriter(csvfile, fieldnames=field)
        writer.writeheader()
        for e in data:
            row = dict()
            row['time'] = float(e.header.stamp.sec)+int(e.header.stamp.nanosec/1000000)*1e-3
            row['pos_x'] = e.point.x
            row['pos_y'] = e.point.y
            writer.writerow(row)

def csv_save_path(filename, data):
    with open(filename, 'w') as csvfile:
        field = ['time', 'pos_x', 'pos_y','yaw','pitch','roll']
        writer = csv.DictWriter(csvfile, fieldnames=field)
        writer.writeheader()
        for e in data.poses:
            row = dict()
            row['time'] = float(e.header.stamp.sec)+int(e.header.stamp.nanosec/1000000)*1e-3
            row['pos_x'] = e.pose.position.x
            row['pos_y'] = e.pose.position.y
            q = cs.Quaternion(e.pose.orientation.w,
                e.pose.orientation.x,
                e.pose.orientation.y,
                e.pose.orientation.z
                            )
            row['yaw'] = q.euler().yaw
            row['pitch'] = q.euler().pitch
            row['roll'] = q.euler().roll
            writer.writerow(row)

class DataSaverNode(Node):

    def __init__(self):
        super().__init__('data_saver')
        self._path_sub = self.create_subscription(Path, '/path', self._path_callback, 10)
        self._modified_path_sub = self.create_subscription(Path, '/modified_path', self._modified_path_callback, 10)
        self.srv = self.create_service(Empty, '/data_saver/save_trajectories', self.save_trajectories_callback)
        self.client = self.create_client(Empty, '/map_saver/save_map')
        self.request = Empty.Request()

        self._path_data = Path()
        self._modified_path_data = Path()

    def _path_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self._path_data = msg

    def _modified_path_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self._modified_path_data = msg

    def save_trajectories_callback(self, request, response):
        self.get_logger().info("Save data to CSV files")
        self.save_to_csv()
        return response

    def send_request(self):
        print("save map request sent")
        self.client.call_async(self.request)

    def save_to_csv(self):
        if(len(self._path_data.poses) > 0):
            csv_save_path("csv/path.csv",self._path_data)
            self.get_logger().info("SLAM path data saved")
        if(len(self._modified_path_data.poses) > 0):
            csv_save_path("csv/modified_path.csv",self._modified_path_data)
            self.get_logger().info("SLAM corrected path data saved")


def main(args=None):
    rclpy.init(args=args)
    node = DataSaverNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
