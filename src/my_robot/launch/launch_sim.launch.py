import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

import yaml



def generate_launch_description():
    package_name='my_robot'
    
    with open(os.path.join(get_package_share_directory(package_name), 'config', 'launch_sim.config.yaml'), 'r') as f:
        launch_config = yaml.safe_load(f)
    
    
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),
                    launch_config['rsp']['launch_directory'],
                    launch_config['rsp']['launch_file']
                )]),launch_arguments={'use_sim_time': launch_config['rsp']['use_sim_time']}.items()
    )
    
    world_file = os.path.join(
        get_package_share_directory(package_name),
        launch_config['world_file']['world_directory'],
        launch_config['world_file']['world_file']
    )


    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                launch_arguments={'world': world_file}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')
                        
                        
    nav2 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')]),
                launch_arguments={
                    'use_sim_time': launch_config['nav2']['use_sim_time'],
                    'params_file': os.path.join(
                        get_package_share_directory('nav2_bringup'),
                        'params',
                        launch_config['nav2']['params_file']
                    )
                }.items()
             )

    rviz_config = os.path.join(
        get_package_share_directory(package_name),
        launch_config['rviz']['rviz_directory'],
        launch_config['rviz']['rviz_file']
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    map_path = os.path.join(
        get_package_share_directory(package_name),
        launch_config['map_server']['map_directory']
    )
    
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': os.path.join(map_path,launch_config['map_server']['map_file'])},
            {'use_sim_time': launch_config['map_server']['use_sim_time']}
        ]
    )
    
    map_server_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        output='screen',
        parameters=[
            {'use_sim_time': launch_config['map_server_lifecycle']['use_sim_time']},
            {'autostart': launch_config['map_server_lifecycle']['autostart']},
            {'node_names': launch_config['map_server_lifecycle']['node_names']}
        ]
    )
    
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        output='screen',
        parameters=[
            {'use_sim_time': launch_config['amcl']['use_sim_time']}
        ]
    )
    
    amcl_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        output='screen',
        parameters=[
            {'use_sim_time': launch_config['amcl_lifecycle']['use_sim_time']},
            {'autostart': launch_config['amcl_lifecycle']['autostart']},
            {'node_names': launch_config['amcl_lifecycle']['node_names']}
        ]
    )
    
    map_server_after_spawn = RegisterEventHandler(
    OnProcessExit(
        target_action=spawn_entity,
        on_exit=[map_server, map_server_lifecycle, amcl, amcl_lifecycle]
        )
    )


    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        rviz,
        map_server_after_spawn,
        nav2,
    ])

