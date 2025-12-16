from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get the workspace path
    # Try to find the workspace root by going up from COLCON_PREFIX_PATH (which points to install/)
    colcon_prefix = os.environ.get('COLCON_PREFIX_PATH', '')
    if colcon_prefix:
        # COLCON_PREFIX_PATH is like /path/to/workspace/install, so go up one level
        install_path = colcon_prefix.split(':')[0]
        workspace_path = os.path.dirname(install_path)
    else:
        # Fallback to default workspace location
        workspace_path = os.path.expanduser('~/change_planner_ws')
    
    # Build the correct path to building map
    building_map_path = os.path.join(
        workspace_path,
        'src/demonstrations/rmf_demos/rmf_demos_maps/maps/four_d/four_d.building.yaml'
    )
    
    # Verify file exists, if not try alternative
    if not os.path.exists(building_map_path):
        # Try absolute path
        building_map_path = os.path.expanduser('~/change_planner_ws/src/demonstrations/rmf_demos/rmf_demos_maps/maps/four_d/four_d.building.yaml')
    
    building_map_path_arg = DeclareLaunchArgument(
        'building_map_path',
        default_value=building_map_path,
        description='Path to building map YAML file'
    )
    
    level_name_arg = DeclareLaunchArgument(
        'level_name',
        default_value='L1',
        description='Level name in building map'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Frame ID for markers'
    )
    
    # Static transform publisher: map -> L1
    # This ensures RViz can display markers in 'map' frame
    static_transform = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '0', '0', '0', '0', '0', '0',
            'map', 'L1'
        ],
        output='screen'
    )
    
    # Zone visualizer node
    zone_visualizer = Node(
        package='rmf_visualization_zones',
        executable='zone_visualizer',
        name='zone_visualizer',
        parameters=[{
            'building_map_path': LaunchConfiguration('building_map_path'),
            'level_name': LaunchConfiguration('level_name'),
            'frame_id': LaunchConfiguration('frame_id'),
            'line_width': 0.2,  # Thicker lines for better visibility
            'alpha': 1.0,  # Fully opaque for better visibility
            'waypoint_scale': 0.3  # Small waypoints to match navgraph
        }],
        output='screen'
    )
    
    return LaunchDescription([
        building_map_path_arg,
        level_name_arg,
        frame_id_arg,
        static_transform,
        zone_visualizer
    ])


