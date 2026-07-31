import os

from setuptools import find_packages, setup


package_name = "target_vision_pkg"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md", "UPSTREAM_README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="orangepi",
    maintainer_email="orangepi@todo.todo",
    description="Backup concentric-ring and cross target detector publishing local fine_data.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "target_vision_node = target_vision_pkg.node:main",
        ],
    },
)
