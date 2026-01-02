from setuptools import setup

package_name = 'basic_communication'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    py_modules=[
        'publisher_subscriber_example',
        'service_example',
        'action_example'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/examples', [
            'publisher_subscriber_example.py',
            'service_example.py',
            'action_example.py'
        ]),
        ('share/' + package_name, ['README.md'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Physical AI Author',
    maintainer_email='author@physical-ai.org',
    description='Basic ROS 2 communication examples for the Physical AI & Humanoid Robotics book',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_subscriber_example = publisher_subscriber_example:main',
            'service_example = service_example:main',
            'action_example = action_example:main',
        ],
    },
)