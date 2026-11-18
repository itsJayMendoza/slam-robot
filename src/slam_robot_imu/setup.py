from setuptools import setup

package_name = "slam_robot_imu"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/imu.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="shichiro",
    maintainer_email="cjay.m.mendoza@gmail.com",
    description="MPU6050 IMU driver for the Pi's I2C bus",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "imu_node = slam_robot_imu.imu_node:main",
            "imu_check = slam_robot_imu.cli:main",
        ],
    },
)
