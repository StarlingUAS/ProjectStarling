from setuptools import setup

package_name = "example_controller_python"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",["resource/"+package_name]),
        ("share/" + package_name, ['package.xml'])
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robert Clarke",
    maintainer_email="TODO",
    description="Example Python controller for PX4",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "controller = example_controller_python.controller:main"
        ]
    }
)