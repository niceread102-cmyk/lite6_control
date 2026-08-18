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
        ('share/' + package_name + '/launch', ['launch/lite6.launch.py', 'launch/lite6_custom_ik.launch.py', 'launch/lite6_servo_only.launch.py', 'launch/lite6_real.launch.py', 'launch/lite6_sim.launch.py']),
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
            'lite6_control = my_lite6_control.lite6_control:main',
            'webcam        = my_lite6_control.webcam:main',
            'moveit2       = my_lite6_control.moveit2:main',
            'moveit2_servo = my_lite6_control.moveit2_servo:main',
            'moveit2_cartesian = my_lite6_control.moveit2_cartesian:main',
            'simple_detection = my_lite6_control.simple_detection:main',
            'ik_node       = my_lite6_control.ik_node:main',
            'moveit2_test  = my_lite6_control.moveit2_test:main',
            'sequence         = my_lite6_control.sequence:main',
            'sequence_sim     = my_lite6_control.sequence_sim:main',
            'sequence_sim_cartesian = my_lite6_control.sequence_sim_cartesian:main',
            'sequence_sim_custom_ik = my_lite6_control.sequence_sim_custom_ik:main',
            'sequence_real    = my_lite6_control.sequence_real:main',
            'target_6d_auto = my_lite6_control.target_6d_auto:main',
            'target_6d_auto_real = my_lite6_control.target_6d_auto_real:main',
            'ik_node_moveit   = my_lite6_control.ik_node_moveit:main',
            'ik_node_ur5e  = my_lite6_control.ik_node_ur5e:main',
            'test_cam = my_lite6_control.test_cam:main',
            'obj_6d   = my_lite6_control.obj_6d:main',
            'joint6   = my_lite6_control.joint6:main',
            'joint4   = my_lite6_control.joint4:main',
            'auto_pick_loop = my_lite6_control.auto_pick_loop:main',
        ],
    },
)
