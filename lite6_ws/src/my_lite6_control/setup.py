from glob import glob

from setuptools import find_packages, setup

package_name = 'my_lite6_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/lite6_custom_ik.launch.py', 'launch/lite6_real.launch.py', 'launch/lite6_sim.launch.py']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/robots', glob('config/robots/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joon',
    maintainer_email='joon@mellowingfactory.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'moveit2       = my_lite6_control.moveit2:main',
            'moveit2_servo = my_lite6_control.moveit2_servo:main',
            'moveit2_cartesian = my_lite6_control.moveit2_cartesian:main',
            'moveit2_test  = my_lite6_control.moveit2_test:main',
            'moveit2_action = my_lite6_control.moveit2_action:main',
            'simple_detection = my_lite6_control.simple_detection:main',
            'sequence         = my_lite6_control.sequence:main',
            'sequence_sim     = my_lite6_control.sequence_sim:main',
            'sequence_sim_custom_ik = my_lite6_control.sequence_sim_custom_ik:main',
            'sequence_real    = my_lite6_control.sequence_real:main',
            'sequence_sim_action  = my_lite6_control.sequence_sim_action:main',
            'sequence_sim_action_fix  = my_lite6_control.sequence_sim_action_fix:main',
            'sequence_real_action = my_lite6_control.sequence_real_action:main',
            'target_6d_auto_real = my_lite6_control.target_6d_auto_real:main',
            'ik_node_moveit   = my_lite6_control.ik_node_moveit:main',
            'ik_node_ur5e  = my_lite6_control.ik_node_ur5e:main',
            'ik_node_twist = my_lite6_control.ik_node_twist:main',
            'joint6   = my_lite6_control.joint6:main',
            'joint4   = my_lite6_control.joint4:main',
            'auto_pick_loop = my_lite6_control.auto_pick_loop:main',
            'auto_pick_loop_sim = my_lite6_control.auto_pick_loop_sim:main',
            'custom_ik_rotation = my_lite6_control.custom_ik_rotation:main',
            'custom_ik = my_lite6_control.custom_ik:main',
            'view_joint = my_lite6_control.view_joint:main',
        ],
    },
)
