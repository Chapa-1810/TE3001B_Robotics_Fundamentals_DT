import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('reto_final'),
        'config',
        'params.yaml'
    )
    
    signal_generator_node = Node(
        package='reto_final',
        name='signal_generator_node',
        executable='signal_generator',
        output='screen',
        parameters = [config]
    )

    set_point_node = Node(
        package='reto_final',
        name='set_point_node',
        executable='set_point',
        output='screen',
        parameters = [config]
    )

    # rqt_graph_node = Node(
    #     package='rqt_graph',
    #     executable='rqt_graph',
    #     output='screen',
    # )
    
    # rqt_graph_plot = Node(
    #     package='rqt_plot',
    #     executable='rqt_plot',
    #     output='screen',
    # )
    
    l_d = LaunchDescription([signal_generator_node, set_point_node])

    

    return l_d
