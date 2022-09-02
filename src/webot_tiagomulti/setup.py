from setuptools import setup

package_name = 'webot_tiagomulti'
data_files = []
data_files.append(
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/multirobot_launch.py']))
data_files.append(('share/' + package_name + '/resource', [
     'resource/tiago_webots.urdf',
     'resource/ros2_control.yml',
     'resource/default.rviz',
     'resource/map.pgm',
     'resource/map.yaml',
]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/default.wbt', 'worlds/.default.wbproj']))

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files= data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    maintainer='aayush',
    maintainer_email='5160727@studenti.unige.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    keywords=['ROS', 'Webots', 'Robot', 'Simulation', 'Examples', 'TIAGo'],
    tests_require=['pytest'],
    entry_points={
        'launch.frontend.launch_extension': ['launch_ros = launch_ros'],
        'console_scripts': [
            'tiagobot = webot_tiagomulti.tiago_iron:main',
            'tiagobot1 = webot_tiagomulti.tiago_iron1:main',
        ],
    },
)
