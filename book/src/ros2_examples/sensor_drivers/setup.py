from setuptools import setup

package_name = 'sensor_drivers'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    py_modules=['sensor_driver_framework'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/examples', [
            'sensor_driver_framework.py'
        ])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Physical AI Author',
    maintainer_email='author@physical-ai.org',
    description='ROS 2 sensor driver framework for the Physical AI & Humanoid Robotics book',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_driver_framework = sensor_driver_framework:main',
        ],
    },
)