import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mobile_robotics_autonomous'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))), 
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
    ],
    install_requires=['setuptools','custom_interfaces'],
    zip_safe=True,
    maintainer='ricard',
    maintainer_email='ric4rd11@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_line_traffic = mobile_robotics_autonomous.move_line_traffic:main',
            'move_points_traffic = mobile_robotics_autonomous.move_points_traffic:main',
            'odom = mobile_robotics_autonomous.odom:main',
            'streetSign_YOLOv8 = mobile_robotics_autonomous.streetSign_YOLOv8:main',
            'stopLight_YOLOv8 = mobile_robotics_autonomous.stopLight_YOLOv8:main'
        ],
    },
)
