from setuptools import setup

package_name = 'unity_tf_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/unity_sim.launch.py',
            'launch/unity_nav2.launch.py',
        ]),
        ('lib/' + package_name, [
            'scripts/odom_to_tf',
            'scripts/map_relay',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Bridge for Unity simulation to ROS2 TF tree',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_to_tf = unity_tf_bridge.odom_to_tf:main',
            'map_relay = unity_tf_bridge.map_relay:main',
        ],
    },
)