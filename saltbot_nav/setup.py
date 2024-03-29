from setuptools import find_packages, setup

package_name = 'saltbot_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/saltbot_visual.launch.xml',
          'launch/localization_launch.py',
          'launch/navigation_launch.py',
          'launch/jackal_nav.launch.py',
          'launch/start_3d_slam.launch.xml',
          'launch/velodyne.launch.py',
          'launch/filtered_velodyne.launch.py',
          'launch/rviz_launch.py']),
        ('share/' + package_name + '/config',
         ['config/nav2_config.rviz', 'config/saltbot.rviz',
          'config/nav2_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sags',
    maintainer_email='srikanthschelbert2024@u.northwestern.edu',
    description='Allows a robot to spread salt using SLAM',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigator2 = saltbot_nav.navigator2:main',
            'laser2grid = saltbot_nav.laser2grid:main',
            'map_slicer = saltbot_nav.map_slicer:main',
            'map_publisher = saltbot_nav.map_publisher:main'
        ],
    },
)
