import os

from setuptools import find_packages, setup


package_name = 'thermal_camera'


def package_files(directory):
    data_files = []
    for path, _, filenames in os.walk(directory):
        if not filenames:
            continue
        install_path = os.path.join('share', package_name, path)
        files = [os.path.join(path, filename) for filename in filenames]
        data_files.append((install_path, files))
    return data_files


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
    ] + package_files('launch') + package_files('config'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thesamayan',
    maintainer_email='user@todo.com',
    description='ROS 2 thermal camera node for the Topdon TC001.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'thermal_camera_node = thermal_camera.thermal_camera_node:main',
        ],
    },
)
