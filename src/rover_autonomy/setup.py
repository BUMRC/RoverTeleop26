from glob import glob

from setuptools import find_packages, setup


package_name = "rover_autonomy"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/config", glob("config/*.json")),
        ("share/" + package_name + "/config", glob("config/*.md")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="parallels",
    maintainer_email="parallels@todo.todo",
    description="Rover autonomy utilities and launch files.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "goal_bridge = rover_autonomy.goal_bridge:main",
            "nav_cmd_bridge = rover_autonomy.nav_cmd_bridge:main",
            "odom_relay = rover_autonomy.odom_relay:main",
        ],
    },
)
