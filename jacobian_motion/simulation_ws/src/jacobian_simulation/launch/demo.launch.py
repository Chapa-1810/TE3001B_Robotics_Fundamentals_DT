import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = '3dof.urdf.xml'
    urdf = os.path.join(
        get_package_share_directory('jacobian_simulation'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
        
    rviz_file = "3dofConfig.rviz"
    rviz_file_path = os.path.join(
        get_package_share_directory('jacobian_simulation'),
        rviz_file)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            arguments=[urdf],
            ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file_path],
            ),
        
        
    ])
    
    """Node(
            package='jacobian_simulation',
            executable='state_publisher',
            name='state_publisher',
            output='screen'),"""