from asyncio.log import logger
import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch.actions
import launch_ros
import yaml
import rclpy

##################################################

### Launch Parameters ###

## Yaml param file

yaml_param_file = os.path.join(get_package_share_directory('olcmr_pkg'), 'params/robot_configs', 'newer_college_2021.yaml')

with open(yaml_param_file, "r") as file_handle:
    param_file = yaml.load(file_handle, Loader=yaml.FullLoader)

## General parameters
use_ekf_prior = param_file.get('use_ekf_prior')
use_slam_loop_closure = param_file.get('use_slam_loop_closure')
compute_camera_lidar_projection = param_file.get('compute_camera_lidar_projection')
number_of_cameras = param_file.get('number_of_cameras')
save_trajectories = param_file.get('save_trajectories')
visualise = param_file.get('visualise')
rviz_config_file = os.path.join(get_package_share_directory('olcmr_pkg'),'params/rviz',param_file.get('rviz_config_file'))
downsample_points_cloud = param_file.get('downsample_points_cloud')
vg_size_for_slam = param_file.get('vg_size_for_slam')
vg_size_for_tsdf = param_file.get('vg_size_for_tsdf')
camera_data_encoding = "mono8"

## Parameters and calibration files
ekf_param_file = os.path.join(get_package_share_directory('olcmr_pkg'),'params/ekf', param_file.get('ekf_param_file'))
lidarslam_param_file = os.path.join(get_package_share_directory('olcmr_pkg'),'params/lidarslam', param_file.get('lidarslam_param_file'))
cameras_calibration_file = os.path.join(get_package_share_directory('olcmr_pkg'),'params/calibration', param_file.get('cameras_calibration_file'))

## Data bag
use_bag = param_file.get('use_bag')
use_ros1_bag = param_file.get('use_ros1_bag')
bag_file = param_file.get('bag_file')
if(use_ros1_bag):
    bag_file = "-s rosbag_v2 " + bag_file

## ROS2 Topics
lidar_point_cloud_topic = param_file.get('lidar_point_cloud_topic')
imu_topic = param_file.get('imu_topic')
kinematic_odom_topic = param_file.get('kinematic_odom_topic')
camera1_namespace = param_file.get('camera1_namespace')
camera2_namespace = param_file.get('camera2_namespace')
camera3_namespace = param_file.get('camera3_namespace')

## Frames
map_frame = param_file.get('map_frame')
lidar_frame = param_file.get('lidar_frame')
camera1_frame = param_file.get('camera1_frame')
camera2_frame = param_file.get('camera2_frame')
camera3_frame = param_file.get('camera3_frame')
slam_frame = param_file.get('slam_frame')
ekf_frame = param_file.get('ekf_frame')

## Tf
publish_static_tf = param_file.get("publish_static_tf")
lidar_tf = param_file.get("lidar_tf")
camera_tf = param_file.get("imu_camera_tf")
use_manually_defined_cameras_tf = param_file.get("use_manually_defined_cameras_tf")
camera1_tf = param_file.get("camera1_tf")
camera2_tf = param_file.get("camera2_tf")
camera3_tf = param_file.get("camera3_tf")

##################################################
##################################################

### Nodes definitions ###

def generate_launch_description():

    # play the data bag
    bag_param_path = os.path.join(get_package_share_directory('olcmr_pkg'),'params/bag_qos/bag_param.yaml')
    play_mapping_bag = launch.actions.ExecuteProcess(
        cmd=['ros2', 'bag', 'play','--qos-profile-overrides-path ', bag_param_path, bag_file,
             '-r','1.0','-d','1.0','--clock'],
        output='log',
        shell=True)

    # aggregate the LiDAR data into a point cloud
    velodyne_pointcloud_node = launch_ros.actions.Node(
        name='velodyne_pointcloud_node',
        executable='velodyne_convert_node',
        package='velodyne_pointcloud',
        arguments=['--ros-args', '-p', 'use_sim_time:='+(str)(use_bag)],
        parameters=[{'min_range': 0.8,
                     'max_range': 50.0,
                     'view_direction': 0.0,
                     'view_width': 5.23598775598}],
        output='both',
        remappings = [('velodyne_points',lidar_point_cloud_topic)]
    )

    # downsample the LiDAR point cloud (if required)
    point_cloud_downsampler = launch_ros.actions.Node(
        name='point_cloud_downsampler',
        executable='pc_downsampler',
        package='olcmr_pkg',
        arguments=['--ros-args', '-p', 'use_sim_time:='+(str)(use_bag)],
        parameters=[{'vg_size_for_slam': vg_size_for_slam},
                    {'vg_size_for_tsdf': vg_size_for_tsdf}],
        output='both',
        remappings = [('point_cloud',lidar_point_cloud_topic),
                      ('point_cloud/downsampled/slam',lidar_point_cloud_topic+"/downsampled/slam"),
                      ('point_cloud/downsampled/tsdf',lidar_point_cloud_topic+"/downsampled/tsdf")]
    )

    ## static tf between robot base and LiDAR
    lidar_tf.extend([slam_frame, lidar_frame,'--ros-args', '-p', 'use_sim_time:='+(str)(use_bag)])
    lidar_static_tf = launch_ros.actions.Node(
        name='lidar_static_tf',
        executable='static_transform_publisher',
        package='tf2_ros',
        arguments=lidar_tf,
        output='both',
    )

    ## static tf between lidar and first camera
    camera_tf.extend([slam_frame, camera1_frame,'--ros-args', '-p', 'use_sim_time:='+(str)(use_bag)])
    camera_static_tf = launch_ros.actions.Node(
        name='camera_static_tf',
        executable='static_transform_publisher',
        package='tf2_ros',
        arguments=camera_tf,

        output='both',
    )

    ## static tf between robot base and cameras (if the cameras tf are defined manually outside of the calibration file)
    camera1_tf.extend([slam_frame, camera1_frame,'--ros-args', '-p', 'use_sim_time:='+(str)(use_bag)])
    camera1_static_tf = launch_ros.actions.Node(
        name='camera1_static_tf',
        executable='static_transform_publisher',
        package='tf2_ros',
        arguments=camera1_tf,
        output='both',
    )
    camera2_tf.extend([slam_frame, camera2_frame,'--ros-args', '-p', 'use_sim_time:='+(str)(use_bag)])
    camera2_static_tf = launch_ros.actions.Node(
        name='camera2_static_tf',
        executable='static_transform_publisher',
        package='tf2_ros',
        arguments=camera2_tf,
        output='both',
    )
    camera3_tf.extend([slam_frame, camera3_frame,'--ros-args', '-p', 'use_sim_time:='+(str)(use_bag)])
    camera3_static_tf = launch_ros.actions.Node(
        name='camera3_static_tf',
        executable='static_transform_publisher',
        package='tf2_ros',
        arguments=camera3_tf,
        output='both',
    )

    ## Newer college custom EKF prior
    imu_static_tf = launch_ros.actions.Node(
        name='imu_static_tf',
        executable='static_transform_publisher',
        package='tf2_ros',
        arguments=['0.014', '-0.012', '-0.015', '0','0','0', ekf_frame, "os_imu",'--ros-args', '-p', 'use_sim_time:='+(str)(use_bag)],
        output='both',
    )

    ## EKF fusion between IMU and kinematic odom for localisation prior
    ekf_localization = launch_ros.actions.Node(
        package='robot_localization',
        name='ekf_odom_imu',
        executable='ekf_node',
        parameters=[ekf_param_file],
        output='both',
    )

    ## matching point cloud / camera image to produce a colored point cloud
    if(downsample_points_cloud):
        tsdf_point_cloud_topic = lidar_point_cloud_topic+"/downsampled/tsdf"
    else:
        tsdf_point_cloud_topic = lidar_point_cloud_topic
    lidar_cameras_projection = launch_ros.actions.Node(
        package='olcmr_pkg',
        name='lidar_cameras_projection',
        executable='lidar_cameras_projection.py',
        output='both',
        arguments=['--ros-args', '-p', 'use_sim_time:='+(str)(use_bag)],
        remappings=[('point_cloud',tsdf_point_cloud_topic),
                    ('point_cloud/colored',lidar_point_cloud_topic+'/colored')],
        parameters=[{'nb_cameras': number_of_cameras,
                     'calibration_file': cameras_calibration_file,
                     'lidar_frame':lidar_frame,
                     'use_manually_defined_cameras_tf':use_manually_defined_cameras_tf,
                     'camera1_frame':camera1_frame,
                     'camera2_frame':camera2_frame,
                     'camera3_frame':camera3_frame,
                     'camera_data_encoding': camera_data_encoding}],
    )

    ## cameras info publication
    images_undistortion = launch_ros.actions.Node(
        package='olcmr_pkg',
        name='images_undistortion',
        executable='images_undistortion.py',
        output='both',
        arguments=['--ros-args', '-p', 'use_sim_time:='+(str)(use_bag)],
        remappings=[('image1_raw/compressed',camera1_namespace+"/compressed"),
                    ('image2_raw/compressed',camera2_namespace+"/compressed"),
                    ('image3_raw/compressed',camera3_namespace+"/compressed"),
                    ('image1_raw/undistorded',camera1_namespace+"/undistorded"),
                    ('image2_raw/undistorded',camera2_namespace+"/undistorded"),
                    ('image3_raw/undistorded',camera3_namespace+"/undistorded"),
                    ('camera1_info',camera1_namespace+"/camera_info"),
                    ('camera2_info',camera2_namespace+"/camera_info"),
                    ('point_cloud',lidar_point_cloud_topic),
                    ('camera3_info',camera3_namespace+"/camera_info")],
        parameters=[{'nb_cameras': number_of_cameras,
                     'calibration_file': cameras_calibration_file,
                     'camera_data_encoding': camera_data_encoding}],
    )

    ros1_bridge = launch_ros.actions.Node(
        package='ros1_bridge',
        name='ros1_bridge',
        executable='dynamic_bridge',
        output='log',
    )

    ############################################
    ### lidarslam_ros2 by GitHub user rsasaki0109 ###
    lidarslam_param = launch.substitutions.LaunchConfiguration(
        'main_param_dir',
        default=lidarslam_param_file)

    ## front-end SLAM node
    if(downsample_points_cloud):
        slam_point_cloud_topic = lidar_point_cloud_topic+"/downsampled/slam"
    else:
        slam_point_cloud_topic = lidar_point_cloud_topic
    scanmatcher = launch_ros.actions.Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        arguments=['--ros-args', '-p', 'use_sim_time:='+(str)(use_bag)],
        parameters=[lidarslam_param],
        remappings=[('input_cloud',slam_point_cloud_topic),
                    ('imu',imu_topic)],
        output='screen'
        )

    ## back-end SLAM node
    graphbasedslam = launch_ros.actions.Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        arguments=['--ros-args', '-p', 'use_sim_time:='+(str)(use_bag)],
        parameters=[lidarslam_param],
        output='screen'
        ) 
    ############################################

    ## trajectories saving for plotting with csv/data_plot.py
    data_saver_node = launch_ros.actions.Node(
        package='olcmr_pkg',
        name='data_saver',
        executable='data_saver.py',
        output='both',
    )

    ## Visualisation on Rviz2
    rviz = launch_ros.actions.Node(
        package='rviz2',
        name='rviz2',
        executable='rviz2',
        arguments=['-d',rviz_config_file],
        output='both',
    )

##################################################

    node_list = [scanmatcher]
    if(publish_static_tf):
        node_list.append(lidar_static_tf)
    if(downsample_points_cloud):
        node_list.append(point_cloud_downsampler)
    if(use_ekf_prior):
        node_list.append(ekf_localization)
        node_list.append(imu_static_tf)
    if(use_bag):
        node_list.append(play_mapping_bag)
    if(use_slam_loop_closure):
        node_list.append(graphbasedslam)
    if(compute_camera_lidar_projection):
        node_list.append(lidar_cameras_projection)
        node_list.append(images_undistortion)
        node_list.append(ros1_bridge)
        if(use_manually_defined_cameras_tf and publish_static_tf):
            node_list.append(camera1_static_tf)
            if(number_of_cameras>1):
                node_list.append(camera2_static_tf)
            if(number_of_cameras>2):
                node_list.append(camera3_static_tf)
        elif(publish_static_tf):
            node_list.append(camera_static_tf)
    if(save_trajectories):
        node_list.append(data_saver_node)
    if(visualise):
        node_list.append(rviz)

    logger = rclpy.logging.get_logger("olcmr_log")
    logger.info("OLCMR launched")
    logger.info("---------------------")
    logger.info("SLAM EKF prior: " + (str)(use_ekf_prior))
    logger.info("SLAM loop-closure: " + (str)(use_slam_loop_closure))
    logger.info("Camera LiDAR projection: " + (str)(compute_camera_lidar_projection))
    if(compute_camera_lidar_projection):
        logger.info("Number of cameras: " + (str)(number_of_cameras))
        logger.info("/!\\ Source ROS1 then ROS2 when using the ROS1 bridge, be sure to have a roscore running /!\\")
    logger.info("Save trajectories: " + (str)(save_trajectories))
    logger.info("Visualise on Rviz: " + (str)(visualise))
    logger.info("Downsample point cloud: " + (str)(downsample_points_cloud))
    if(downsample_points_cloud):
        logger.info("SLAM downsampling voxel size: " + (str)(vg_size_for_slam))
        logger.info("TSDF downsampling voxel size: " + (str)(vg_size_for_tsdf))
    logger.info("Use ROS1 bag: " + (str)(use_ros1_bag))
    if(use_ros1_bag):
        logger.info("/!\\ Source ROS1 then ROS2 when using a ROS1 bag /!\\")
    logger.info("---------------------")
    

    return launch.LaunchDescription(node_list)
