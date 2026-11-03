"""ament_python package mss_teleop (ADR-031): teleop launch, joystick command node, configs."""

from glob import glob

from setuptools import setup

PACKAGE = "mss_teleop"

setup(
    name=PACKAGE,
    version="0.0.1",
    packages=[PACKAGE],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE}"]),
        (f"share/{PACKAGE}", ["package.xml"]),
        (f"share/{PACKAGE}/launch", glob("launch/*.launch.py")),
        (f"share/{PACKAGE}/config", glob("config/*.yaml")),
        (f"share/{PACKAGE}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="multi-sensor-slam maintainers",
    maintainer_email="maintainers@multi-sensor-slam.invalid",
    description="Teleop and live SLAM view for the multi-sensor-slam simulation (ADR-031).",
    license="TODO",
    entry_points={"console_scripts": [f"joy_commands = {PACKAGE}.joy_commands:main"]},
)
