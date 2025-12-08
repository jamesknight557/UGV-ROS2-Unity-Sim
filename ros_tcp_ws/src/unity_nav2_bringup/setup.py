from setuptools import setup

package_name = 'unity_nav2_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/unity_nav2_bringup.launch.py']),
        ('share/' + package_name + '/config', ['config/nav2_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='james',
    maintainer_email='james@example.com',
    description='Nav2 bringup for Unity UGV',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)

