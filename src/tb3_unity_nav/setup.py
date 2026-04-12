from setuptools import setup

package_name = 'tb3_unity_nav'

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
        ('share/' + package_name + '/param', [
            'param/unity_nav2.yaml',
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
    description='TurtleBot3 Unity simulation Nav2 bridge',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_to_tf = tb3_unity_nav.odom_to_tf:main',
            'map_relay = tb3_unity_nav.map_relay:main',
        ],
    },
)