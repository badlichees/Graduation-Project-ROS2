from setuptools import find_packages, setup

package_name = 'diff_drive_controller'

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
    description='Unity 差速驱动机器人速度指令转换节点',
    license='MIT',
    entry_points={
        'console_scripts': [
            'diff_drive_controller = diff_drive_controller.diff_drive_controller:main',
        ],
    },
)
