import launch
import launch.actions
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os

def generate_launch_description():
    # Read in the vehicle's namespace through the command line or use the default value one is not provided
    launch.actions.DeclareLaunchArgument(
        "robot", 
        default_value="tempest",
        description="Name of the vehicle",
    )

    robot = 'tempest'
    # declare the path to the robot's vehicle description file
    config = os.path.join(
        get_package_share_directory('riptide_descriptions2'),
        'config',
        robot + '.yaml'
    )

    return launch.LaunchDescription([
        # create the nodes    
        launch_ros.actions.Node(
            package='riptide_teleop2',
            executable='ps3_teleop',
            name='riptide_teleop2',
            respawn=True,
            output='screen',
            
            parameters=[{"vehicle_config": config}]
        )
    ])