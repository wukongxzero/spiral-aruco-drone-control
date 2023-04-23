from launch import LaunchDescription
from launch_ros.actions import IncludeLaunchDescription, Node
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Find the path to the px4_ros_com package
    PX4_Autopilot_dir = get_package_share_directory('PX4-Autopilot')

    # Include the px4.launch.xml file
    px4 = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PX4_Autopilot_dir + '/launch/px4.launch'),
    )

    # Start the mavsdk_ros node
    mavsdk_ros = Node(
        package='mavsdk_ros',
        executable='mavsdk_ros',
        name='mavsdk_ros_node',
        output='screen',
        parameters=[{'connection_url': 'udp://:14540'}],
    )

    # Start the go_to_spiralplusaruco_node
    aruco_node = Node(
        package='my_drone_package',
        executable='aruco_node.py',
        name='aruco_node',
        output='screen',
    )
    # Create the launch description object and add the nodes
    ld = LaunchDescription()
    ld.add_action(px4)
    ld.add_action(mavsdk_ros)
    ld.add_action(aruco_node)

    return ld
