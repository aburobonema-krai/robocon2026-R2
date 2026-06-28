from setuptools import setup

package_name = 'krai_s3_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rifat nur',
    maintainer_email='moh.fahri12018@gmail.com',
    description='Serial bridge between ROS2 Humble and ESP32-S3.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            's3_bridge_node = krai_s3_bridge.s3_bridge_node:main',
        ],
    },
)
