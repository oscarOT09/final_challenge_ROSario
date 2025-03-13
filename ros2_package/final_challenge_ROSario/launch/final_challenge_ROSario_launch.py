from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    
    sp_node = Node(name="set_point_node_ROSario",
                       package='final_challenge_ROSario',
                       executable='set_point',
                       output='screen',
                       )
    
    rqt_plot_node = ExecuteProcess(
        cmd = ['ros2', 'run', 'rqt_plot', 'rqt_plot'],
        name = 'rqt_plot',
        output = 'screen'
    )
    rqt_graph_node = ExecuteProcess(
        cmd = ['ros2', 'run', 'rqt_graph', 'rqt_graph'],
        name = 'rqt_graph',
        output = 'screen'
    )
    rqt_reconfigure_node = ExecuteProcess(
        cmd = ['ros2', 'run', 'rqt_reconfigure', 'rqt_reconfigure'],
        name = 'rqt_reconfigure',
        output = 'screen'
    )

    micro_ros_agent = ExecuteProcess(
        cmd = ['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/ttyUSB0'],
        name = 'micro_ros_agent',
        output = 'screen'
    )

    l_d = LaunchDescription([sp_node, rqt_plot_node, rqt_graph_node, rqt_reconfigure_node, micro_ros_agent])

    return l_d