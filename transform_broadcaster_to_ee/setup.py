from setuptools import find_packages, setup

package_name = 'transform_broadcaster_to_ee'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'geometry_msgs',
        'tf2_ros',
        'numpy',
        'tf_transformations',
    ],
    zip_safe=True,
    maintainer='nam017',
    maintainer_email='karlym.nam@data61.csiro.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #'transform_broadcaster_to_ee_node=transform_broadcaster_to_ee:transform_broadcaster_to_ee_node:main'
            'broadcaster_node = transform_broadcaster_to_ee.transform_broadcaster_to_ee_node:main',
            'test_broadcaster_node = transform_broadcaster_to_ee.test_broadcaster:main'
        ],
    },
)
