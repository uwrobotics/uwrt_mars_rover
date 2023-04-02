from setuptools import setup

package_name = 'uwrt_mars_rover_led_matrix'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='',
    maintainer_email='',
    description='',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'led_matrix = uwrt_mars_rover_led_matrix.rgb_pins::main'
        ],
    },

    data_files=[
        # Add launch file
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ]
)