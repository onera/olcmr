#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import point_cloud2 as pc2
import time
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import csv
import numpy as np

logger = rclpy.logging.get_logger("trajectory_replayer_node")

def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    
    return [qx, qy, qz, qw]

class PointCloudDownSample(Node):

    def __init__(self):
        super().__init__('trajectory_replay')
        self.declare_parameter('trajectory_csv_file', 'csv/path.csv')
        trajectory_csv_file = self.get_parameter('trajectory_csv_file').value
        self._sub_point_cloud = self.create_subscription(PointCloud2,'point_cloud',self._point_cloud_cb,10)
        self.pub_point_cloud_ = self.create_publisher(PointCloud2, 'point_cloud/synchronised', 10)
        self.br = TransformBroadcaster(self)
        self.pose_list = self.read_csv_file(trajectory_csv_file)

    def _point_cloud_cb(self, msg):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'slam_base_link'

        current_time = float(msg.header.stamp.sec)+int(msg.header.stamp.nanosec/1000000)*1e-3
        closest_pose = np.argmin(np.abs(np.ones(len(self.pose_list['time']))*current_time - self.pose_list['time']))
        if(abs(current_time - self.pose_list['time'][closest_pose]) > 0.0001):
            return

        t.transform.translation.x = self.pose_list['pos_x'][closest_pose]
        t.transform.translation.y = self.pose_list['pos_y'][closest_pose]
        t.transform.translation.z = 0.0

        q = get_quaternion_from_euler(
            self.pose_list['roll'][closest_pose], 
            self.pose_list['pitch'][closest_pose], 
            self.pose_list['yaw'][closest_pose])
        
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.br.sendTransform(t)
        self.pub_point_cloud_.publish(msg)

    def read_csv_file(self,trajectory_csv_file):
        fields = ['time', 'pos_x', 'pos_y','yaw','pitch','roll']
        data = dict()
        for f in fields:
            data[f] = []
        with open(trajectory_csv_file) as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                for f in fields:
                    data[f].append(float(row[f]))
        return data

    import numpy as np # Scientific computing library for Python

def main(args=None):

    rclpy.init(args=args)
    node = PointCloudDownSample()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()