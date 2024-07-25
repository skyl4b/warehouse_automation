from pathlib import Path
from warnings import simplefilter

from setuptools import SetuptoolsDeprecationWarning, find_packages, setup

# Suppress the deprecation warning for setuptools,
# there's no alternative as this is the tooling for ROS2 Humble still
simplefilter("ignore", category=SetuptoolsDeprecationWarning)

name = "wa_bringup"
source_dir = Path(__file__).parent

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
        (
            f"share/{name}/launch",
            [str(path) for path in (source_dir / "launch").iterdir()],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Eduardo Farinati Leite",
    maintainer_email="eduardo.farinati@ufrgs.br",
    description="Bringup for the warehouse automation project",
    license="MIT",
    entry_points={
        "console_scripts": [f"box_spawner = {name}.box_spawner:main"],
    },
)
