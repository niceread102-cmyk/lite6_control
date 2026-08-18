#!/usr/bin/env python3
import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import load_yaml, generate_ros2_control_params_temp_file


def launch_setup(context, *args, **kwargs):
    dof = LaunchConfiguration('dof', default=6)
    robot_type = LaunchConfiguration('robot_type', default='lite')
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='ufactory')
    limited = LaunchConfiguration('limited', default=True)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=True)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='uf_robot_hardware/UFRobotFakeSystemHardware')
    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)

    controllers_name = 'fake_controllers'
    xarm_type = '{}{}'.format(robot_type.perform(context), dof.perform(context))

    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm_controller'), 'config', '{}_controllers.yaml'.format(xarm_type)),
        prefix=prefix.perform(context),
        add_gripper=add_gripper.perform(context) in ('True', 'true'),
        ros_namespace=ros_namespace,
        robot_type=robot_type.perform(context),
    )

    moveit_config = MoveItConfigsBuilder(
        context=context,
        controllers_name=controllers_name,
        dof=dof,
        robot_type=robot_type,
        prefix=prefix,
        hw_ns=hw_ns,
        limited=limited,
        effort_control=effort_control,
        velocity_control=velocity_control,
        ros2_control_plugin=ros2_control_plugin,
        ros2_control_params=ros2_control_params,
        add_gripper=add_gripper,
        add_vacuum_gripper=add_vacuum_gripper,
    ).to_moveit_configs()

    robot_description_parameters = {}
    robot_description_parameters.update(moveit_config.robot_description)
    robot_description_parameters.update(moveit_config.robot_description_semantic)
    robot_description_parameters.update(moveit_config.robot_description_kinematics)
    robot_description_parameters.update(moveit_config.joint_limits)
    robot_description_parameters.update(moveit_config.planning_pipelines)

    servo_yaml = load_yaml('xarm_moveit_servo', 'config/xarm_moveit_servo_config.yaml')
    servo_yaml['move_group_name'] = xarm_type
    xarm_traj_controller = '{}{}_traj_controller'.format(prefix.perform(context), xarm_type)
    servo_yaml['command_out_topic'] = '/{}/joint_trajectory'.format(xarm_traj_controller)
    servo_params = {'moveit_servo': servo_yaml}

    container = ComposableNodeContainer(
        name='lite6_servo_container',
        namespace='/',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='moveit_servo',
                plugin='moveit_servo::ServoNode',
                name='servo_server',
                parameters=[
                    servo_params,
                    robot_description_parameters,
                ],
            ),
        ],
        output='screen',
    )

    return [container]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('add_gripper', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
