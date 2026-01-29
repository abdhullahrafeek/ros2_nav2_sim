import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit



def generate_launch_description():
    package_name='my_robot'
    
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    world_file = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'my_world.world'
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
                    'use_sim_time': 'true',
                    'params_file': os.path.join(
                        get_package_share_directory('nav2_bringup'),
                        'params',
                        'nav2_params.yaml'
                    )
                }.items()
             )

    rviz_config = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'my_robot_rviz.rviz'
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[
            {'yaml_filename':'my_new_map_save.yaml'},
            {'use_sim_time':True}
        ]
    )
    
    map_server_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        output='screen',
        parameters=[
            {'use_sim_time':True},
            {'autostart':True},
            {'node_names':['map_server']}
        ]
    )
    
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        output='screen',
        parameters=[
            {'use_sim_time':True}
        ]
    )
    
    amcl_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        output='screen',
        parameters=[
            {'use_sim_time':True},
            {'autostart':True},
            {'node_names':['amcl']}
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

