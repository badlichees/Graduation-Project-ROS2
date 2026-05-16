from setuptools import find_packages, setup

package_name = 'ping_responder'

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
    description='延迟测试用 Pong 消息响应节点',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ping_responder = ping_responder.ping_responder:main',
        ],
    },
)
