from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sentinel_sensor_console'

setup(
    name=package_name,
    version='0.0.0',
    # find_packages() buscará automáticamente las subcarpetas que tengan un __init__.py (como widgets)
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sahid',
    maintainer_email='tu_correo@ejemplo.com',
    description='Consola local de monitoreo de sensores y algoritmos para la Jetson',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Aquí le decimos a ROS que el comando "console" ejecutará la función main() del archivo main.py
            'console = sentinel_sensor_console.main:main'
        ],
    },
)