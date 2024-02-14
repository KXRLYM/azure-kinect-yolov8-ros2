from setuptools import find_packages, setup

package_name = 'ros2_pcl_distance'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nam017',
    maintainer_email='karlym.nam@data61.csiro.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_point_cloud_distance = ros2_pcl_distance.ros2_point_cloud_distance:main',
            'ros2_point_cloud_only = ros2_pcl_distance.point_cloud_only:main',
            'ros2_point_cloud_rgb = ros2_pcl_distance.point_cloud_rgb:main',
        ],
    },
)
