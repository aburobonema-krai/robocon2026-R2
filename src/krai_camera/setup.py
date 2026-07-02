from setuptools import setup

package_name = 'krai_camera'

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
    description='YOLO-based camera detection node for KRAI robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'krai_camera_node = krai_camera.camera_detection_node:main',
        ],
    },
)
