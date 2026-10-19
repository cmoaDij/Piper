from glob import glob

from setuptools import find_packages, setup


PACKAGE_NAME = "piper_policy_bridge"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=False,
    maintainer="Piper Project",
    maintainer_email="maintainers@example.invalid",
    description="Safety-bounded ROS bridge for Piper LeRobot inference",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "piper_policy_bridge = piper_policy_bridge.ros_node:main",
            "piper_command_mux = piper_policy_bridge.command_mux:main",
            "piper_home_guard = piper_policy_bridge.home_guard:main",
            "piper_gripper_calibrate_closed = "
            "piper_policy_bridge.gripper_calibrate_guard:main",
            "piper_policy_reset = piper_policy_bridge.reset_policy:main",
        ]
    },
)
