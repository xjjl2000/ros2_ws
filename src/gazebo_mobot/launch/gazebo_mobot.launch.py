"""Demo for using ros2 launch command to start a simulation world file.

Launches Gazebo with a specified world file.

"""

import os

from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import ThisLaunchFileDir

from launch_ros.actions import Node
from launch.conditions import IfCondition

from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration



def generate_launch_description():

    robot_name = 'mobot_final.urdf'

    world_file_name = 'guandao.world'

    urdf = os.path.join('/home/change/ros2_ws/src/gazebo_mobot', 'urdf', robot_name)

    world = os.path.join('/home/change/ros2_ws/src/gazebo_mobot', 'worlds', world_file_name)

        # read urdf contents because to spawn an entity in 
    # gazebo we need to provide entire urdf as string on  command line

    xml = open(urdf, 'r').read()
 
    # double quotes need to be with escape sequence
    xml = xml.replace('"', '\\"')
 
    # this is argument format for spwan_entity service 
    spwan_args = '{name: \"mobot\", xml: \"'  +  xml + '\" }'

 
    # spawn_mobot = Node(
    #     package='gazebo_ros',
    #     node_executable='spawn_entity.py',
    #     node_name='spawn_entity',
    #     #node_namespace=namespace_,
    #     #emulate_tty=True,
    #     arguments=['-entity',
    #                'car_description',
    #                '-x', '3.5', '-y', '1.0', '-z', '0.1',
    #                '-file', urdf
    #                ],
    #     output='screen'
    # )



    # gazebo = ExecuteProcess(cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], output='screen')

    # return LaunchDescription([

    # gazebo

    # ])

    # create and return launch description object
    return LaunchDescription([

        # start gazebo, notice we are using libgazebo_ros_factory.so instead of libgazebo_ros_init.so
        # That is because only libgazebo_ros_factory.so contains the service call to /spawn_entity
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so'],
            output='screen'),
 
        # tell gazebo to spwan your robot in the world by calling service
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', spwan_args],
            output='screen'),

        # spawn_mobot,
    ])