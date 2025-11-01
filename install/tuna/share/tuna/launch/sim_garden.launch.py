from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    # Package directory
    pkg_tuna = get_package_share_directory('tuna')
    
    # Try to use source world file first (for development)
    # If not found, fall back to installed world file
    source_world_file = os.path.join(os.path.dirname(__file__), '..', '..', 'tuna', 'worlds', 'simWorld.sdf')
    source_world_file = os.path.abspath(source_world_file)
    
    if os.path.exists(source_world_file):
        world_file = source_world_file
        print(f"Using source world file: {world_file}")
    else:
        world_file = os.path.join(pkg_tuna, 'worlds', 'simWorld.sdf')
        print(f"Using installed world file: {world_file}")
    
    # Try to use source URDF file first (for development)
    source_urdf_file = os.path.join(os.path.dirname(__file__), '..', '..', 'tuna', 'urdf', 'zpm_robot.urdf.xacro')
    source_urdf_file = os.path.abspath(source_urdf_file)
    
    if os.path.exists(source_urdf_file):
        urdf_file = source_urdf_file
    else:
        urdf_file = os.path.join(pkg_tuna, 'urdf', 'zpm_robot.urdf.xacro')
    
    # Process xacro file
    robot_description_content = xacro.process_file(urdf_file).toxml()
    
    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Path to the Gazebo world file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    x_arg = DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='Initial x position'
    )
    
    y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='Initial y position'
    )
    
    z_arg = DeclareLaunchArgument(
        'z',
        default_value='0.3',
        description='Initial z position'
    )
    
    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Initial yaw angle'
    )
    
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_description_content
        }],
        output='screen'
    )
    
    # Gazebo Garden simulator
    gazebo_sim = ExecuteProcess(
        cmd=['gz', 'sim', world_file],
        output='screen'
    )
    
    # Note: Robot can be spawned manually via Gazebo Garden GUI:
    # 1. Click "Insert" in the GUI
    # 2. Navigate to your robot model
    # Or use gz service command manually after launch:
    # gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --req "sdf_filename: '/path/to/robot.urdf' name: 'zpm_robot'"
    
    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        robot_state_publisher_node,
        gazebo_sim,
    ])
