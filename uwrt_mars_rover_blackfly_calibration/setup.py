from setuptools import setup
import os
from glob import glob
package_name = 'uwrt_mars_rover_blackfly_calibration'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.xml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='younes',
    maintainer_email='yreda@uwaterloo.ca',
    description='Package for calibrating blackfly cameras',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'broadcaster = uwrt_mars_rover_blackfly_calibration.broadcaster:main'
        ],
    },
)
