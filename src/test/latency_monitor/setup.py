from setuptools import find_packages, setup

package_name = 'latency_monitor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liuzhili',
    maintainer_email='liuzhili86@gmail.com',
    description='ROS2 与 Unity 间通信延迟监测节点',
    license='MIT',
    entry_points={
        'console_scripts': [
            'latency_monitor = latency_monitor.latency_monitor:main',
        ],
    },
)
