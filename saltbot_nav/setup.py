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
            'navigator = saltbot_nav.navigator:main',
            'laser2grid = saltbot_nav.laser2grid:main',
            'map_slicer = saltbot_nav.map_slicer:main',
            'map_publisher = saltbot_nav.map_publisher:main'
        ],
    },
)
