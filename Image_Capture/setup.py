from setuptools import find_packages, setup

package_name = 'Image_Capture'

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
    'sensor_msgs',
    'cv_bridge',
    'opencv-python'
    ],
    zip_safe=True,
    maintainer='nam017',
    maintainer_email='karlym.nam@data61.csiro.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_capture_node = Image_Capture.image_capture_node:main'
        ],
    },
)
