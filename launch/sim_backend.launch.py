from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # bridge gazebo services (world: override)
    gz_services_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='entity_services_bridge',
        arguments=[
            '/world/override/remove@ros_gz_interfaces/srv/DeleteEntity',
            '/world/override/create@ros_gz_interfaces/srv/SpawnEntity',
            '/world/override/set_pose@ros_gz_interfaces/srv/SetEntityPose'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # robot model bridges
    otto_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='otto_bridge',
        arguments=['model/Otto/pose@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V'],
        remappings=[('/model/Otto/pose', '/otto_pose')]
    )

    # element pose tracking
    object_poses = Node(
        package='override_sim',
        executable='pose_bridge.py'
    )

    # field locations
    locator = Node(
        package='override_sim',
        executable='field_location.py'
    )

    # world services endpoint
    world_services = Node(
        package='override_sim',
        executable='world_services.py'
    )

    # scoring the game
    scoring = Node(
        package='override_sim',
        executable='scoring.py',
        output='screen'
    )

    return LaunchDescription([
        otto_pose_bridge,
        object_poses,
        gz_services_bridge,
        scoring,
        world_services,
        locator,
    ])
