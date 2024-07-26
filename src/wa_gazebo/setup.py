import warnings

from setuptools import find_packages, setup
from setuptools.command.easy_install import EasyInstallDeprecationWarning
from setuptools.warnings import SetuptoolsDeprecationWarning

# Ignore setuptools and easy_install deprecation warning
warnings.filterwarnings("ignore", category=SetuptoolsDeprecationWarning)
warnings.filterwarnings("ignore", category=EasyInstallDeprecationWarning)

# Setup package
name = "wa_gazebo"

setup(
    name=name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{name}"],
        ),
        (f"share/{name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "wa_interfaces", "wa_environment"],
    zip_safe=True,
    maintainer="Eduardo Farinati Leite",
    maintainer_email="eduardo.farinati@ufrgs.br",
    description=(
        "Gazebo simulation connector for the warehouse automation project"
    ),
    license="MIT",
    entry_points={
        "console_scripts": [
            f"gazebo_bridge = {name}.bridge:main",
        ],
    },
)
