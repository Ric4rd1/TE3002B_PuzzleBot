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
    maintainer_email='ric4rd11@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cv_bridge_example = mobile_robotics_cv.cv_bridge_example:main',
            'color_detector = mobile_robotics_cv.color_detector:main',
            'odom = mobile_robotics_cv.odom:main',
            'move_test = mobile_robotics_cv.move_test:main',
            'move_test2 = mobile_robotics_cv.move_test2:main',
            'move_test3 = mobile_robotics_cv.move_test3:main',
            'move_points = mobile_robotics_cv.move_points:main',
            'point_generator = mobile_robotics_cv.point_generator:main',
            'stopLight_detector = mobile_robotics_cv.stopLight_detector:main',
            'hsv_tunner = mobile_robotics_cv.hsv_tunner:main',
            'move_points_traffic = mobile_robotics_cv.move_points_traffic:main',
        ],
    },
)
