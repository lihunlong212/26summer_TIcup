from setuptools import setup


package_name = "domain_bridge_pkg"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="orangepi",
    maintainer_email="orangepi@todo.todo",
    description="ROS Domain 10 to Domain 1 flight choice and telemetry bridge.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "domain_bridge = domain_bridge_pkg.domain_bridge:main",
        ],
    },
)
