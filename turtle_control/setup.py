from setuptools import find_packages, setup

package_name = 'turtle_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'launch/start_turtle_control.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='redhairedlynx',
    maintainer_email='pushkardave.vnit@gmail.com',
    description='Package to control the turtle',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint = turtle_control.waypoint_node:main'
        ],
    },
)
