#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, PointField
import cv2
import numpy as np
from cv_bridge import CvBridge
from tf2_ros import TransformListener, Buffer
import math
import struct
import time
import yaml
import sys
from collections import namedtuple
import ctypes
import PyKDL

logger = rclpy.logging.get_logger("camera_log")

class LidarCameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')
        self.declare_parameter('nb_cameras', 3)
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('lidar_frame', 'lidar')
        self.declare_parameter('use_manually_defined_cameras_tf', False)
        self.declare_parameter('camera1_frame', '')
        self.declare_parameter('camera2_frame', '')
        self.declare_parameter('camera3_frame', '')
        self.declare_parameter('camera_data_encoding', 'bgr8')

        self.nb_cameras = self.get_parameter('nb_cameras').value
        with open(self.get_parameter('calibration_file').value) as f:
            self.calibration = yaml.load(f, Loader=yaml.FullLoader)
        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.camera1_frame = self.get_parameter('camera1_frame').value
        self.camera2_frame = self.get_parameter('camera2_frame').value
        self.camera3_frame = self.get_parameter('camera3_frame').value
        self.use_manually_defined_cameras_tf = self.get_parameter('use_manually_defined_cameras_tf').value
        self.camera_data_encoding = self.get_parameter('camera_data_encoding').value

        self.camera1_topic = self.calibration.get('cam0').get('rostopic')+'/undistorded'
        if(self.nb_cameras>1):
            self.camera2_topic = self.calibration.get('cam1').get('rostopic')+'/undistorded'
        if(self.nb_cameras>2):
            self.camera3_topic = self.calibration.get('cam2').get('rostopic')+'/undistorded'
        self.pub_point_cloud_ = self.create_publisher(PointCloud2, 'point_cloud/colored', 10)
        self.tfBuffer = Buffer()
        self.listener = TransformListener(self.tfBuffer, self)


        # intrinsect camera1 matrix
        self.fx1 = self.calibration.get('cam0').get('intrinsics')[0]
        self.fy1 = self.calibration.get('cam0').get('intrinsics')[1]
        self.cu1 = self.calibration.get('cam0').get('intrinsics')[2]
        self.cv1 = self.calibration.get('cam0').get('intrinsics')[3]
        self.int_mat1 = np.array([[self.fx1,0,self.cu1],
                                 [0,self.fy1,self.cv1],
                                 [0,0,1]])

        # intrinsect camera2 matrix
        if(self.nb_cameras>1):
            self.fx2 = self.calibration.get('cam1').get('intrinsics')[0]
            self.fy2 = self.calibration.get('cam1').get('intrinsics')[1]
            self.cu2 = self.calibration.get('cam1').get('intrinsics')[2]
            self.cv2 = self.calibration.get('cam1').get('intrinsics')[3]
            self.int_mat2 = np.array([[self.fx2,0,self.cu2],
                                    [0,self.fy2,self.cv2],
                                    [0,0,1]])

        # intrinsect camera3 matrix
        if(self.nb_cameras>2):
            self.fx3 = self.calibration.get('cam2').get('intrinsics')[0]
            self.fy3 = self.calibration.get('cam2').get('intrinsics')[1]
            self.cu3 = self.calibration.get('cam2').get('intrinsics')[2]
            self.cv3 = self.calibration.get('cam2').get('intrinsics')[3]
            self.int_mat3 = np.array([[self.fx3,0,self.cu3],
                                    [0,self.fy3,self.cv3],
                                    [0,0,1]])


        # calculation of the transform matrix lidar -> camera1
        while(not self.tfBuffer.can_transform(self.camera1_frame,self.lidar_frame,self.get_clock().now())) :
            time.sleep(0.5)
            rclpy.spin_once(self)
        tr = self.tfBuffer.lookup_transform(self.camera1_frame,self.lidar_frame,self.get_clock().now())
        [r,p,y] = euler_from_quaternion(tr.transform.rotation.x, tr.transform.rotation.y, tr.transform.rotation.z, tr.transform.rotation.w)
        trans_mat = np.array([[1,0,0,tr.transform.translation.x],
                            [0,1,0,tr.transform.translation.y],
                            [0,0,1,tr.transform.translation.z],
                            [0,0,0,1]])
        rot_mat_z = np.array([[math.cos(y),-math.sin(y),0,0],
                            [math.sin(y),math.cos(y),0,0],
                            [0,0,1,0],
                            [0,0,0,1]])
        rot_mat_y = np.array([[math.cos(p),0,math.sin(p),0],
                            [0,1,0,0],
                            [-math.sin(p),0,math.cos(p),0],
                            [0,0,0,1]])
        rot_mat_x = np.array([[1,0,0,0],
                            [0,math.cos(r),-math.sin(r),0],
                            [0,math.sin(r),math.cos(r),0],
                            [0,0,0,1]])
        ext_mat =  trans_mat @ rot_mat_z @ rot_mat_y @ rot_mat_x
        self.ext_mat1 = np.array([ext_mat[0],ext_mat[1],ext_mat[2]])
        self.proj_mat1 = self.int_mat1 @ self.ext_mat1

        # calculation of the transform matrix lidar -> camera2
        if(self.nb_cameras>1):
            if(self.use_manually_defined_cameras_tf):
                while(not self.tfBuffer.can_transform(self.camera2_frame,self.lidar_frame,self.get_clock().now())) :
                    time.sleep(0.5)
                    rclpy.spin_once(self)
                tr = self.tfBuffer.lookup_transform(self.camera2_frame,self.lidar_frame,self.get_clock().now())
                [r,p,y] = euler_from_quaternion(tr.transform.rotation.x, tr.transform.rotation.y, tr.transform.rotation.z, tr.transform.rotation.w)
                trans_mat = np.array([[1,0,0,tr.transform.translation.x],
                                    [0,1,0,tr.transform.translation.y],
                                    [0,0,1,tr.transform.translation.z],
                                    [0,0,0,1]])
                rot_mat_z = np.array([[math.cos(y),-math.sin(y),0,0],
                                    [math.sin(y),math.cos(y),0,0],
                                    [0,0,1,0],
                                    [0,0,0,1]])
                rot_mat_y = np.array([[math.cos(p),0,math.sin(p),0],
                                    [0,1,0,0],
                                    [-math.sin(p),0,math.cos(p),0],
                                    [0,0,0,1]])
                rot_mat_x = np.array([[1,0,0,0],
                                    [0,math.cos(r),-math.sin(r),0],
                                    [0,math.sin(r),math.cos(r),0],
                                    [0,0,0,1]])
                ext_mat =  trans_mat @ rot_mat_z @ rot_mat_y @ rot_mat_x
                self.ext_mat2 = np.array([ext_mat[0],ext_mat[1],ext_mat[2]])
                self.proj_mat2 = self.int_mat2 @ self.ext_mat2

            else:
                ext_mat2 = self.calibration.get('cam1').get('T_cn_cnm1') @ ext_mat
                self.ext_mat2 = np.array([ext_mat2[0],ext_mat2[1],ext_mat2[2]])
                self.proj_mat2 = self.int_mat2 @ self.ext_mat2

        # calculation of the transform matrix lidar -> camera3
        if(self.nb_cameras>2):
            if(self.use_manually_defined_cameras_tf):
                while(not self.tfBuffer.can_transform(self.camera3_frame,self.lidar_frame,self.get_clock().now())) :
                    time.sleep(0.5)
                    rclpy.spin_once(self)
                tr = self.tfBuffer.lookup_transform(self.camera3_frame,self.lidar_frame,self.get_clock().now())
                [r,p,y] = euler_from_quaternion(tr.transform.rotation.x, tr.transform.rotation.y, tr.transform.rotation.z, tr.transform.rotation.w)
                trans_mat = np.array([[1,0,0,tr.transform.translation.x],
                                    [0,1,0,tr.transform.translation.y],
                                    [0,0,1,tr.transform.translation.z],
                                    [0,0,0,1]])
                rot_mat_z = np.array([[math.cos(y),-math.sin(y),0,0],
                                    [math.sin(y),math.cos(y),0,0],
                                    [0,0,1,0],
                                    [0,0,0,1]])
                rot_mat_y = np.array([[math.cos(p),0,math.sin(p),0],
                                    [0,1,0,0],
                                    [-math.sin(p),0,math.cos(p),0],
                                    [0,0,0,1]])
                rot_mat_x = np.array([[1,0,0,0],
                                    [0,math.cos(r),-math.sin(r),0],
                                    [0,math.sin(r),math.cos(r),0],
                                    [0,0,0,1]])
                ext_mat =  trans_mat @ rot_mat_z @ rot_mat_y @ rot_mat_x
                self.ext_mat3 = np.array([ext_mat[0],ext_mat[1],ext_mat[2]])
                self.proj_mat3 = self.int_mat3 @ self.ext_mat3
            else:
                ext_mat3 = self.calibration.get('cam2').get('T_cn_cnm1') @ ext_mat2
                self.ext_mat3 = np.array([ext_mat3[0],ext_mat3[1],ext_mat3[2]])
                self.proj_mat3 = self.int_mat3 @ self.ext_mat3

        self._sub_point_cloud = self.create_subscription(PointCloud2,'point_cloud',self._point_cloud_cb,10)
        self._sub_camera1 = self.create_subscription(Image,self.camera1_topic,self._camera1_cb,10)
        self.image1 = Image()
        self.bridge = CvBridge()
        if(self.nb_cameras>1):
            self._sub_camera2 = self.create_subscription(Image,self.camera2_topic,self._camera2_cb,10)
            self.image2 = Image()
        if(self.nb_cameras>2):
            self._sub_camera3 = self.create_subscription(Image,self.camera3_topic,self._camera3_cb,10)
            self.image3 = Image()
        
        self.point_cloud  = PointCloud2()
        logger.info("Transform matrix initialised")


    def _point_cloud_cb(self, msg):
        self.point_cloud = msg
        if( self.image1.encoding == '' or 
            (self.nb_cameras > 1 and self.image2.encoding == '') or 
            (self.nb_cameras > 2 and self.image3.encoding == '')):
            logger.info("At least 1 image topic is empty, retrying...")
            return
        # converting image to openCV matrix
        cv_image1 = self.bridge.imgmsg_to_cv2(self.image1)
        height1=cv_image1.shape[1]
        width1=cv_image1.shape[0]
        if(self.nb_cameras>1):
            cv_image2 = self.bridge.imgmsg_to_cv2(self.image2)
            height2=cv_image2.shape[1]
            width2=cv_image2.shape[0]
        if(self.nb_cameras>2):
            cv_image3 = self.bridge.imgmsg_to_cv2(self.image3)
            height3=cv_image3.shape[1]
            width3=cv_image3.shape[0]

        lidar_points = []

        # point cloud formatting
        point_list = read_points_list(msg, field_names=["x","y","z"],skip_nans=True)
        point_array = np.array(point_list)
        coords = np.concatenate((point_array,np.ones((len(point_array),1))),axis=1)
        point_array1 = point_array[(self.ext_mat1 @ coords.T)[2]>0]      # remove points behind image plane
        coords1 = coords[(self.ext_mat1 @ coords.T)[2]>0] 

        projection1 = (self.proj_mat1 @ coords1.T).T                      # project points on plane
        projection1[:,0] = projection1[:,0]/projection1[:,2]
        projection1[:,1] = projection1[:,1]/projection1[:,2]
        image_points1 = np.concatenate((projection1[:,0:2],
                                     (self.ext_mat1 @ coords1.T)[2].reshape(len(projection1),1)),axis=1)         # u,v,z_c
        point_array1 = point_array1[image_points1[:,0]>=0]                  # check if points are in image bounds
        image_points1 = image_points1[image_points1[:,0]>=0]                  # check if points are in image bounds
        point_array1 = point_array1[image_points1[:,0]<height1-1]
        image_points1 = image_points1[image_points1[:,0]<height1-1]
        point_array1 = point_array1[image_points1[:,1]>=0]
        image_points1 = image_points1[image_points1[:,1]>=0]
        point_array1 = point_array1[image_points1[:,1]<width1-1]
        image_points1 = image_points1[image_points1[:,1]<width1-1]

        v1 = image_points1[:,0].astype(int)
        u1 = image_points1[:,1].astype(int)
        # 3D points => 2D pixels matching
        for i in range(u1.shape[0]) :
            u = u1[i]
            v = v1[i]
            # point cloud coloration
            if(self.camera_data_encoding == "mono8"):
                cam_data = struct.unpack('I', struct.pack('BBBB', 
                                                            (int)(cv_image1[u,v]), 
                                                            (int)(cv_image1[u,v]), 
                                                            (int)(cv_image1[u,v]), 255))[0]
            else:
                cam_data = struct.unpack('I', struct.pack('BBBB', 
                                                            (int)(cv_image1[u,v][0]), 
                                                            (int)(cv_image1[u,v][1]), 
                                                            (int)(cv_image1[u,v][2]), 255))[0]
            new_point = [point_array1[i,0],point_array1[i,1],point_array1[i,2],cam_data]
            lidar_points.append(new_point)

        if(self.nb_cameras>1):
            point_array2 = point_array[(self.ext_mat2 @ coords.T)[2]>0]      # remove points behind image plane
            coords2 = coords[(self.ext_mat2 @ coords.T)[2]>0] 

            projection2 = (self.proj_mat2 @ coords2.T).T                      # project points on plane
            projection2[:,0] = projection2[:,0]/projection2[:,2]
            projection2[:,1] = projection2[:,1]/projection2[:,2]
            image_points2 = np.concatenate((projection2[:,0:2],
                                        (self.ext_mat2 @ coords2.T)[2].reshape(len(projection2),1)),axis=1)         # u,v,z_c
            point_array2 = point_array2[image_points2[:,0]>=0]                  # check if points are in image bounds
            image_points2 = image_points2[image_points2[:,0]>=0]                  # check if points are in image bounds
            point_array2 = point_array2[image_points2[:,0]<height2-1]
            image_points2 = image_points2[image_points2[:,0]<height2-1]
            point_array2 = point_array2[image_points2[:,1]>=0]
            image_points2 = image_points2[image_points2[:,1]>=0]
            point_array2 = point_array2[image_points2[:,1]<width2-1]
            image_points2 = image_points2[image_points2[:,1]<width2-1]

            v2 = image_points2[:,0].astype(int)
            u2 = image_points2[:,1].astype(int)
            for i in range(u2.shape[0]) :
                u = u2[i]
                v = v2[i]
                # point cloud coloration
                if(self.camera_data_encoding == "mono8"):
                    cam_data = struct.unpack('I', struct.pack('BBBB', 
                                                                (int)(cv_image2[u,v]), 
                                                                (int)(cv_image2[u,v]), 
                                                                (int)(cv_image2[u,v]), 255))[0]
                else:
                    cam_data = struct.unpack('I', struct.pack('BBBB', 
                                                                (int)(cv_image2[u,v][0]), 
                                                                (int)(cv_image2[u,v][1]), 
                                                                (int)(cv_image2[u,v][2]), 255))[0]
                new_point = [point_array2[i,0],point_array2[i,1],point_array2[i,2],cam_data]
                lidar_points.append(new_point)

        if(self.nb_cameras>2):
            point_array3 = point_array[(self.ext_mat3 @ coords.T)[2]>0]      # remove points behind image plane
            coords3 = coords[(self.ext_mat3 @ coords.T)[2]>0] 

            projection3 = (self.proj_mat3 @ coords3.T).T                      # project points on plane
            projection3[:,0] = projection3[:,0]/projection3[:,2]
            projection3[:,1] = projection3[:,1]/projection3[:,2]
            image_points3 = np.concatenate((projection3[:,0:2],
                                        (self.ext_mat3 @ coords3.T)[2].reshape(len(projection3),1)),axis=1)         # u,v,z_c
            point_array3 = point_array3[image_points3[:,0]>=0]                  # check if points are in image bounds
            image_points3 = image_points3[image_points3[:,0]>=0]                  # check if points are in image bounds
            point_array3 = point_array3[image_points3[:,0]<height3-1]
            image_points3 = image_points3[image_points3[:,0]<height3-1]
            point_array3 = point_array3[image_points3[:,1]>=0]
            image_points3 = image_points3[image_points3[:,1]>=0]
            point_array3 = point_array3[image_points3[:,1]<width3-1]
            image_points3 = image_points3[image_points3[:,1]<width3-1]

            v3 = image_points3[:,0].astype(int)
            u3 = image_points3[:,1].astype(int)
            for i in range(u3.shape[0]) :
                u = u3[i]
                v = v3[i]
                # point cloud coloration
                if(self.camera_data_encoding == "mono8"):
                    cam_data = struct.unpack('I', struct.pack('BBBB', 
                                                                (int)(cv_image3[u,v]), 
                                                                (int)(cv_image3[u,v]), 
                                                                (int)(cv_image3[u,v]), 255))[0]
                else:
                    cam_data = struct.unpack('I', struct.pack('BBBB', 
                                                                (int)(cv_image3[u,v][0]), 
                                                                (int)(cv_image3[u,v][1]), 
                                                                (int)(cv_image3[u,v][2]), 255))[0]
                new_point = [point_array3[i,0],point_array3[i,1],point_array3[i,2],cam_data]
                lidar_points.append(new_point)

        # point cloud publication
        header = msg.header
        point_cloud = create_cloud_xyzrgb32(header, lidar_points)
        self.pub_point_cloud_.publish(point_cloud)

    def _camera1_cb(self, msg):
        self.image1 = msg
    def _camera2_cb(self, msg):
        self.image2 = msg
    def _camera3_cb(self, msg):
        self.image3 = msg


def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z # in radians

### PointCloud2 relative helper functions ###

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a L{sensor_msgs.PointCloud2} message.

    @param cloud: The point cloud to read from.
    @type  cloud: L{sensor_msgs.PointCloud2}
    @param field_names: The names of fields to read. If None, read all fields. [default: None]
    @type  field_names: iterable
    @param skip_nans: If True, then don't return any point with a NaN value.
    @type  skip_nans: bool [default: False]
    @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
    @type  uvs: iterable
    @return: Generator which yields a list of values for each point.
    @rtype:  generator
    """
    assert isinstance(cloud, PointCloud2), 'cloud is not a sensor_msgs.msg.PointCloud2'
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, \
                                                       cloud.point_step, cloud.row_step, \
                                                       cloud.data, math.isnan
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    yield unpack_from(data, offset)
                    offset += point_step

def read_points_list(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a L{sensor_msgs.PointCloud2} message.

    This function returns a list of namedtuples. It operates on top of the read_points method. For more efficient access use read_points directly. 

    @param cloud: The point cloud to read from.
    @type  cloud: L{sensor_msgs.PointCloud2}
    @param field_names: The names of fields to read. If None, read all fields. [default: None]
    @type  field_names: iterable
    @param skip_nans: If True, then don't return any point with a NaN value.
    @type  skip_nans: bool [default: False]
    @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
    @type  uvs: iterable
    @return: List of namedtuples containing the values for each point
    @rtype: list
    """
    assert isinstance(cloud, PointCloud2), 'cloud is not a sensor_msgs.msg.PointCloud2'

    if field_names is None:
        field_names = [f.name for f in cloud.fields]

    Point = namedtuple("Point", field_names)

    return [Point._make(l) for l in read_points(cloud, field_names, skip_nans, uvs)]


def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'
    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print('Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt    += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt

def create_cloud(header, fields, points):
    """
    Create a L{sensor_msgs.msg.PointCloud2} message.

    @param header: The point cloud header.
    @type  header: L{std_msgs.msg.Header}
    @param fields: The point cloud fields.
    @type  fields: iterable of L{sensor_msgs.msg.PointField}
    @param points: The point cloud points.
    @type  points: list of iterables, i.e. one iterable for each point, with the
                   elements of each iterable being the values of the fields for
                   that point (in the same order as the fields parameter)
    @return: The point cloud.
    @rtype:  L{sensor_msgs.msg.PointCloud2}
    """

    cloud_struct = struct.Struct(_get_struct_fmt(False, fields))

    buff = ctypes.create_string_buffer(cloud_struct.size * len(points))

    point_step, pack_into = cloud_struct.size, cloud_struct.pack_into
    offset = 0
    for p in points:
        pack_into(buff, offset, *p)
        offset += point_step

    return PointCloud2(header=header,
                       height=1,
                       width=len(points),
                       is_dense=False,
                       is_bigendian=False,
                       fields=fields,
                       point_step=cloud_struct.size,
                       row_step=cloud_struct.size * len(points),
                       data=buff.raw)

def create_cloud_xyzi32(header, points):
    """
    Create a L{sensor_msgs.msg.PointCloud2} message with 3 float32 fields (x, y, z).

    @param header: The point cloud header.
    @type  header: L{std_msgs.msg.Header}
    @param points: The point cloud points.
    @type  points: iterable
    @return: The point cloud.
    @rtype:  L{sensor_msgs.msg.PointCloud2}
    """
    fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
              PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
              PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
              PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)]
    return create_cloud(header, fields, points)

def create_cloud_xyzrgb32(header, points):
    """
    Create a L{sensor_msgs.msg.PointCloud2} message with 3 float32 fields (x, y, z).

    @param header: The point cloud header.
    @type  header: L{std_msgs.msg.Header}
    @param points: The point cloud points.
    @type  points: iterable
    @return: The point cloud.
    @rtype:  L{sensor_msgs.msg.PointCloud2}
    """
    fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
              PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
              PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
              PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]
    return create_cloud(header, fields, points)


def main(args=None):
    rclpy.init(args=args)

    node = LidarCameraNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()