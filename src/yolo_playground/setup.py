import os

from setuptools import find_packages, setup


package_name = "yolo_playground"


def package_files(directory):
    data_files = []
    for path, _, filenames in os.walk(directory):
        if not filenames:
            continue
        install_path = os.path.join("share", package_name, path)
        files = [os.path.join(path, filename) for filename in filenames]
        data_files.append((install_path, files))
    return data_files


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml", "README.md"]),
    ]
    + package_files("launch")
    + package_files("config")
    + package_files("models"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="thesamayan",
    maintainer_email="samayan@example.com",
    description="Custom package for local YOLO ROS workflows.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "detection_logger = yolo_playground.detection_logger:main",
            "detection_to_tf = yolo_playground.detection_to_tf:main",
            "landmark_saver = yolo_playground.landmark_saver:main",
        ],
    },
)
