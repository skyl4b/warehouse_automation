import warnings

from setuptools import find_packages, setup
from setuptools.command.easy_install import EasyInstallDeprecationWarning
from setuptools.warnings import SetuptoolsDeprecationWarning

# Ignore setuptools and easy_install deprecation warning
warnings.filterwarnings("ignore", category=SetuptoolsDeprecationWarning)
warnings.filterwarnings("ignore", category=EasyInstallDeprecationWarning)

# Setup package
name = "wa_task_transmitter"

setup(
    name=name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    # packages=[package_name],
    # package_dir={package_name: "src"},
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{name}"],
        ),
        ("share/" + name, ["package.xml"]),
    ],
    install_requires=["setuptools", "wa_interfaces"],
    zip_safe=True,
    maintainer="skylab",
    maintainer_email="eduardo.farinati@ufrgs.br",
    description="Transmits tasks for the Warehouse Automation project",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "task_transmitter = src.task_transmitter:main",
            "conveyor_belt = src.conveyor_belt:main",
            "mobile_bot = src.mobile_bot:main",
        ],
    },
)
