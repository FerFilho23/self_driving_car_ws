from setuptools import setup
import os 
from glob import glob

package_name = 'self_driving_car_pkg'
config_module = "self_driving_car_pkg/config" 
detection_module = "self_driving_car_pkg/detection"
lane_detection_module = "self_driving_car_pkg/detection/lanes"

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, config_module, detection_module, lane_detection_module],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*')),
        (os.path.join('share', package_name,'worlds'), glob('worlds/*')),
        (os.path.join('lib', package_name), glob('scripts/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ferfilho',
    maintainer_email='fernando.rsf23@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_recorder_node = self_driving_car_pkg.video_recorder:main',
            'sdf_spawner_node =  self_driving_car_pkg.sdf_spawner:main',
            'computer_vision_node =   self_driving_car_pkg.computer_vision:main'
        ],
    },
)
