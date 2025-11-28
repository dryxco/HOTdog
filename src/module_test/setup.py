from setuptools import setup

package_name = "module_test"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/interface_viewer.launch.py"]),
    ],
    install_requires=["setuptools", "opencv-python"],
    zip_safe=True,
    maintainer="module_test maintainer",
    maintainer_email="user@example.com",
    description="Interface viewer for detection topics.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "interface_viewer = module_test.interface_viewer:main",
        ],
    },
)


