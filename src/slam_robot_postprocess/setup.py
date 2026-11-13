from setuptools import setup

package_name = "slam_robot_postprocess"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="shichiro",
    maintainer_email="cjay.m.mendoza@gmail.com",
    description="Survey post-processing pipeline",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["postprocess = slam_robot_postprocess.pipeline:main"]},
)
