from setuptools import setup

package_name = 'krai_gui'

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
    description='KRAI PyQt5 operator GUI integrated with ROS2 services/actions.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'krai_gui_node = krai_gui.krai_operator_gui:main',
            'gui_placeholder_node = krai_gui.gui_placeholder_node:main',
        ],
    },
)
