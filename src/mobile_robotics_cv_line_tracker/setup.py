import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mobile_robotics_cv_line_tracker'

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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ricard',
    maintainer_email='ric4rd11@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_tracker = mobile_robotics_cv_line_tracker.color_tracker:main',
            'move_line = mobile_robotics_cv_line_tracker.move_line:main',
            'stopLight_detector = mobile_robotics_cv_line_tracker.stopLight_detector:main',
            'move_line_traffic = mobile_robotics_cv_line_tracker.move_line_traffic:main',
            'move_line_trafficV2 = mobile_robotics_cv_line_tracker.move_line_trafficV2:main',
            'hsv_tuner = mobile_robotics_cv_line_tracker.hsv_tunner:main',
            'stopLight_YOLOv8 = mobile_robotics_cv_line_tracker.stopLight_YOLOv8:main',
            'image_data_collector = mobile_robotics_cv_line_tracker.image_data_collector:main'
        ],
    },
)
