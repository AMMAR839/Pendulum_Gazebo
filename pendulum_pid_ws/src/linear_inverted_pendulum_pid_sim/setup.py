from glob import glob
from setuptools import find_packages, setup

package_name = "linear_inverted_pendulum_pid_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (
            f"share/{package_name}",
            ["package.xml", "README.md", "DOKUMENTASI_METODE_KONTROL.txt"],
        ),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/urdf", glob("urdf/*.xacro")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ammar",
    maintainer_email="ammar@example.com",
    description="ROS 2 Jazzy and Gazebo Harmonic PID simulation for a linear inverted pendulum.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "pid_serial_bridge = linear_inverted_pendulum_pid_sim.pid_serial_bridge:main",
        ],
    },
)
