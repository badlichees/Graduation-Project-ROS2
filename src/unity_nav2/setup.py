from setuptools import setup

package_name = 'unity_nav2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/unity_nav2.launch.py',
        ]),
        ('share/' + package_name + '/param', [
            'param/unity_nav2.yaml',
        ]),
        ('share/' + package_name + '/behavior_trees', [
            'behavior_trees/navigate.xml',
        ]),
        ('lib/' + package_name, [
            'scripts/odom_tf_bridge',
            'scripts/map_bridge',
            'scripts/planner_switch',
            'scripts/param_bridge',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liuzhili',
    maintainer_email='liuzhili86@gmail.com',
    description='Unity 仿真与 Nav2 桥接节点集',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odom_tf_bridge = unity_nav2.odom_tf_bridge:main',
            'map_bridge = unity_nav2.map_bridge:main',
            'planner_switch = unity_nav2.planner_switch:main',
            'param_bridge = unity_nav2.param_bridge:main',
        ],
    },
)
