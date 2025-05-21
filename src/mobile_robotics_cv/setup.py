import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mobile_robotics_cv'

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
    maintainer_email='hector@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Nodos existentes
            'test_node = mobile_robotics_cv.test_node:main',
            'pub_sub_with_classes = mobile_robotics_cv.pub_sub_with_classes:main', 
            'move_forward = mobile_robotics_cv.move_forward:main',
            'move_forward_distance = mobile_robotics_cv.move_forward_distance:main',
            'move_agle_given = mobile_robotics_cv.move_angle_given:main',
            'move_square = mobile_robotics_cv.move_square:main',
            'controller_points = mobile_robotics_cv.controller_points:main',
            'path_generator = mobile_robotics_cv.path_generator:main',
            
            # Nodos de seguimiento de línea y detección de esferas
            'odom = mobile_robotics_cv.odom:main',
            'line_follower = mobile_robotics_cv.line_follower:main',
            'sphere_detector = mobile_robotics_cv.sphere_detector:main',
            'line_traffic_controller = mobile_robotics_cv.line_traffic_controller:main',
            'hsv_calibrator = mobile_robotics_cv.hsv_calibrator:main',
            
            # Nodos originales que nos mostraste (si quieres mantenerlos)
            'color_detector = mobile_robotics_cv.color_detector:main',
            'cv_bridge_example = mobile_robotics_cv.cv_bridge_example:main',
            'hsv_tunner = mobile_robotics_cv.hsv_tunner:main',
            'move_points = mobile_robotics_cv.move_points:main',
            'move_points_traffic = mobile_robotics_cv.move_points_traffic:main',
            'move_test = mobile_robotics_cv.move_test:main',
            'move_test2 = mobile_robotics_cv.move_test2:main',
            'move_test3 = mobile_robotics_cv.move_test3:main',
            'point_generator = mobile_robotics_cv.point_generator:main',
            'stopLight_detector = mobile_robotics_cv.stopLight_detector:main',
        ],
    },
)