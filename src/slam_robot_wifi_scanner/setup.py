from setuptools import setup

package_name = "slam_robot_wifi_scanner"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/scanner.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="shichiro",
    maintainer_email="cjay.m.mendoza@gmail.com",
    description="Passive monitor-mode WiFi scanner",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "scanner_node = slam_robot_wifi_scanner.scanner_node:main",
            "wifi_scan_cli = slam_robot_wifi_scanner.cli:main",
            "walk_survey = slam_robot_wifi_scanner.walk_survey:main",
        ],
    },
)
