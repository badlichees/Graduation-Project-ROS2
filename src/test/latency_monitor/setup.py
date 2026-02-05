from setuptools import find_packages, setup

package_name = 'latency_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='badlichees',
    maintainer_email='2442642595@qq.com',
    description='Package for testing latency between ROS2 and Unity',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'latency_monitor = latency_monitor.latency_monitor:main',
        ],
    },
)