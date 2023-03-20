import os
from glob import glob
from setuptools import setup

package_name = 'camera_gimbal_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    # data_files=[
    #     ('share/ament_index/resource_index/packages',
    #         ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    # ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nmolla',
    maintainer_email='nafisfardin@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    #name of the executable, name of the package, name of the file, name of the function we want to start
    entry_points={
        'console_scripts': [
            'talker = camera_gimbal_controller.publisher_member_function:main',
            'listener = camera_gimbal_controller.subscriber_member_function:main',
        ],
    },
    data_files=[
        # ... Other data files
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ]
)
