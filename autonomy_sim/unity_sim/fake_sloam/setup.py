from setuptools import find_packages, setup

package_name = "fake_sloam"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Xu Liu",
    maintainer_email="liuxu@seas.upenn.edu",
    description="The fake_sloam package",
    license="Penn Software License",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fake_sloam_node = fake_sloam.fake_sloam_node:main",
        ],
    },
)
