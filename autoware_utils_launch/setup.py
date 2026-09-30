from setuptools import find_packages
from setuptools import setup

package_name = "autoware_utils_launch"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="Takagi, Isamu",
    maintainer_email="isamu.takagi@tier4.jp",
    license="Apache License 2.0",
    entry_points={
        "launch.frontend.launch_extension": [
            "autoware_utils_launch = autoware_utils_launch",
        ],
    },
)
