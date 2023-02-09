#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, CompressedImage, PointCloud2
import yaml
import cv2
from cv_bridge import CvBridge
import numpy as np

logger = rclpy.logging.get_logger("undistortion_log")

class ImagesUndistortion(Node):

    def __init__(self):
        super().__init__('images_undistortion')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('nb_cameras', 3)
        self.declare_parameter('camera_data_encoding', 'bgr8')
        self.nb_cameras = self.get_parameter('nb_cameras').value
        calibration_file = self.get_parameter('calibration_file').value
        self.camera_data_encoding = self.get_parameter('camera_data_encoding').value
        self.bridge = CvBridge()
        with open(calibration_file, "r") as file_handle:
            calib_data = yaml.load(file_handle, Loader=yaml.FullLoader)

        self.pub_cam1_info_ = self.create_publisher(CameraInfo, 'camera1_info', 10)
        self.pub_image1_undistorded_ = self.create_publisher(Image, 'image1_raw/undistorded', 10)
        self.image1 = CompressedImage()
        self.camera_info_msg1 = CameraInfo()
        self.camera_info_msg1.width = calib_data.get("cam0").get("resolution")[0]
        self.camera_info_msg1.height = calib_data.get("cam0").get("resolution")[1]
        k1 = calib_data.get("cam0").get("intrinsics")
        self.cv_int_mat1 = np.array([[k1[0],0.0,k1[2]],[0.0,k1[1],k1[3]],[0.0,0.0,1.0]])
        self.camera_info_msg1.k = [k1[0],0.0,k1[2],0.0,k1[1],k1[3],0.0,0.0,1.0]
        self.camera_info_msg1.d = calib_data.get("cam0").get("distortion_coeffs")
        self.camera_info_msg1.distortion_model = calib_data.get("cam0").get("distortion_model")
        self._sub_camera1 = self.create_subscription(CompressedImage,'image1_raw/compressed',self._camera1_cb,10)
        if(self.camera_info_msg1.distortion_model == "equidistant"):
            self.map1_cam1, self.map2_cam1 = cv2.fisheye.initUndistortRectifyMap(self.cv_int_mat1, 
                                                            np.array(self.camera_info_msg1.d), 
                                                            np.eye(3), self.cv_int_mat1, 
                                                            (self.camera_info_msg1.width, self.camera_info_msg1.height), 
                                                            cv2.CV_16SC2)

        if(self.nb_cameras>1):
            self.pub_cam2_info_ = self.create_publisher(CameraInfo, 'camera2_info', 10)
            self.pub_image2_undistorded_ = self.create_publisher(Image, 'image2_raw/undistorded', 10)
            self.image2 = CompressedImage()
            self.camera_info_msg2 = CameraInfo()
            self.camera_info_msg2.width = calib_data.get("cam1").get("resolution")[0]
            self.camera_info_msg2.height = calib_data.get("cam1").get("resolution")[1]
            k2 = calib_data.get("cam1").get("intrinsics")
            self.cv_int_mat2 = np.array([[k2[0],0.0,k2[2]],[0.0,k2[1],k2[3]],[0.0,0.0,1.0]])
            self.camera_info_msg2.k = [k2[0],0.0,k2[2],0.0,k2[1],k2[3],0.0,0.0,1.0]
            self.camera_info_msg2.d = calib_data.get("cam1").get("distortion_coeffs")
            self.camera_info_msg2.distortion_model = calib_data.get("cam1").get("distortion_model")
            self._sub_camera2 = self.create_subscription(CompressedImage,'image2_raw/compressed',self._camera2_cb,10)
            if(self.camera_info_msg2.distortion_model == "equidistant"):
                self.map1_cam2, self.map2_cam2 = cv2.fisheye.initUndistortRectifyMap(self.cv_int_mat2, 
                                                                np.array(self.camera_info_msg2.d), 
                                                                np.eye(3), self.cv_int_mat2, 
                                                                (self.camera_info_msg2.width, self.camera_info_msg2.height), 
                                                                cv2.CV_16SC2)

        if(self.nb_cameras>2):
            self.pub_cam3_info_ = self.create_publisher(CameraInfo, 'camera3_info', 10)
            self.pub_image3_undistorded_ = self.create_publisher(Image, 'image3_raw/undistorded', 10)
            self.image3 = CompressedImage()
            self.camera_info_msg3 = CameraInfo()
            self.camera_info_msg3.width = calib_data.get("cam2").get("resolution")[0]
            self.camera_info_msg3.height = calib_data.get("cam2").get("resolution")[1]
            k3 = calib_data.get("cam2").get("intrinsics")
            self.cv_int_mat3 = np.array([[k3[0],0.0,k3[2]],[0.0,k3[1],k3[3]],[0.0,0.0,1.0]])
            self.camera_info_msg3.k = [k3[0],0.0,k3[2],0.0,k3[1],k3[3],0.0,0.0,1.0]
            self.camera_info_msg3.d = calib_data.get("cam2").get("distortion_coeffs")
            self.camera_info_msg3.distortion_model = calib_data.get("cam2").get("distortion_model")
            self._sub_camera3 = self.create_subscription(CompressedImage,'image3_raw/compressed',self._camera3_cb,10)
            if(self.camera_info_msg3.distortion_model == "equidistant"):
                self.map1_cam3, self.map2_cam3 = cv2.fisheye.initUndistortRectifyMap(self.cv_int_mat3, 
                                                                np.array(self.camera_info_msg3.d), 
                                                                np.eye(3), self.cv_int_mat3, 
                                                                (self.camera_info_msg3.width, self.camera_info_msg3.height), 
                                                                cv2.CV_16SC2)

        self._sub_pc = self.create_subscription(PointCloud2,'point_cloud',self._pc_cb,10)

    def _camera1_cb(self, msg):
        self.image1 = msg  

    def _camera2_cb(self, msg):
        self.image2 = msg 

    def _camera3_cb(self, msg):
        self.image3 = msg

    def _pc_cb(self,msg):
        if( self.image1.format == '' or 
            (self.nb_cameras > 1 and self.image2.format == '') or 
            (self.nb_cameras > 2 and self.image3.format == '')):
            logger.info("At least 1 image topic is empty, retrying...")
            return
        cv_image1 = self.bridge.compressed_imgmsg_to_cv2(self.image1)
        if(self.camera_info_msg1.distortion_model == "equidistant"):
            cv_undistorded1 = cv2.remap(cv_image1, self.map1_cam1, self.map2_cam1, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        else :
            cv_undistorded1 = cv2.undistort(cv_image1, self.cv_int_mat1, np.array(self.camera_info_msg1.d))
        undistorded_image1 = self.bridge.cv2_to_imgmsg(cv_undistorded1,self.camera_data_encoding)
        undistorded_image1.header = self.image1.header
        self.pub_image1_undistorded_.publish(undistorded_image1)
        self.camera_info_msg1.header = self.image1.header
        self.pub_cam1_info_.publish(self.camera_info_msg1)
        if(self.nb_cameras>1):
            cv_image2 = self.bridge.compressed_imgmsg_to_cv2(self.image2)
            if(self.camera_info_msg2.distortion_model == "equidistant"):
                cv_undistorded2 = cv2.remap(cv_image2, self.map1_cam2, self.map2_cam2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            else : 
                cv_undistorded2 = cv2.undistort(cv_image2, self.cv_int_mat2, np.array(self.camera_info_msg2.d))
            undistorded_image2 = self.bridge.cv2_to_imgmsg(cv_undistorded2,self.camera_data_encoding)
            undistorded_image2.header = self.image2.header
            self.pub_image2_undistorded_.publish(undistorded_image2)
            self.camera_info_msg2.header = self.image2.header
            self.pub_cam2_info_.publish(self.camera_info_msg2)
        if(self.nb_cameras>2):
            cv_image3 = self.bridge.compressed_imgmsg_to_cv2(self.image3)
            if(self.camera_info_msg3.distortion_model == "equidistant"):
                cv_undistorded3 = cv2.remap(cv_image3, self.map1_cam3, self.map2_cam3, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            else :    
                cv_undistorded3 = cv2.undistort(cv_image3, self.cv_int_mat3, np.array(self.camera_info_msg3.d))
            undistorded_image3 = self.bridge.cv2_to_imgmsg(cv_undistorded3,self.camera_data_encoding)
            undistorded_image3.header = self.image3.header
            self.pub_image3_undistorded_.publish(undistorded_image3)
            self.camera_info_msg3.header = self.image3.header
            self.pub_cam3_info_.publish(self.camera_info_msg3)

def main(args=None):
    rclpy.init(args=args)

    node = ImagesUndistortion()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()