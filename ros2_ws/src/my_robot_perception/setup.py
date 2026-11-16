from glob import glob

from setuptools import setup

package_name = "my_robot_perception"

setup(
    name=package_name,
    version="0.8.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.xml")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Aviral Goyal",
    maintainer_email="aviralgoyalab17@gmail.com",
    description="HSV + depth detectors for the red target cylinder and the green home box",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "target_detector = my_robot_perception.target_detector:main",
            "box_detector = my_robot_perception.box_detector:main",
            "perception_node = my_robot_perception.perception_node:main",
        ],
    },
)
