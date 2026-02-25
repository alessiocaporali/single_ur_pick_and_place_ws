from setuptools import setup, find_packages

package_name = "ur_rtde_interface"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/controller.launch.py"]),
            ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lar",
    maintainer_email="pippo.castoro@nonlibero.it",
    description="UR RTDE ROS2 Python Interface",
    license="MIT",
    entry_points={
        "console_scripts": [
            "robot_controller = ur_rtde_interface.robot_controller:main",
          
        ],
    },
)
