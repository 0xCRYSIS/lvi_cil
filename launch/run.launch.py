import os
from sys import prefix

from pyrfc3339 import generate
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    ####### LIS ##########

    lis_param_file = os.path.join(get_package_share_directory("lvi_sam"),'config','params_lidar.yaml')

    FeatureExtraction_Node = Node(package='lvi_sam',
                                  name='lvi_sam_featureExtraction',
                                  executable='lvi_sam_featureExtraction',
                                  parameters=[lis_param_file])

    ld.add_action(FeatureExtraction_Node)

    ImageProjection_Node = Node(package='lvi_sam',
                                name='lvi_sam_imageProjection',
                                executable='lvi_sam_imageProjection',
                                parameters=[lis_param_file])

    ld.add_action(ImageProjection_Node)

    IMUPreintegration_Node = Node(package='lvi_sam',
                                  name='lvi_sam_imuPreintegration',
                                  executable='lvi_sam_imuPreintegration',
                                  parameters=[lis_param_file])
    
    ld.add_action(IMUPreintegration_Node)

    MapOptimization_Node = Node(package='lvi_sam',
                                name='lvi_sam_mapOptimization',
                                executable='lvi_sam_mapOptimization',
                                parameters=[lis_param_file])
    ld.add_action(MapOptimization_Node)


    ####### VIS ##########

    vis_param_file = os.path.join(get_package_share_directory("lvi_sam"),'config','params_camera.yaml')

    VisualEstimator_Node = Node(package='lvi_sam',
                                name='lvi_sam_visual_odometry',
                                executable='lvi_sam_visual_odometry',
                                parameters=[{"vins_config_file":vis_param_file}],
                                ) #prefix=["xterm -e gdb -ex run --args"]

    ld.add_action(VisualEstimator_Node)

    

    return ld