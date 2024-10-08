from setuptools import find_packages, setup

package_name = 'turtle_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ############################### Begin_Citation [3] ###########################################
        ('share/' + package_name, ['package.xml', 'launch/waypoints.launch.xml', 'config/colors.yaml']),
        ############################### End_Citation [3] ############################################
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
