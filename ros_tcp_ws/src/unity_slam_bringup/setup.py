from setuptools import setup

package_name = 'unity_slam_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch + config
        ('share/' + package_name + '/launch', ['launch/unity_slam_bringup.launch.py']),
        ('share/' + package_name + '/config', ['config/slam_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='james',
    maintainer_email='james@example.com',
    description='Unity → ROS2 SLAM bringup package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_to_tf_broadcaster = unity_slam_bringup.odom_to_tf_broadcaster:main',
        ],
    },
)

