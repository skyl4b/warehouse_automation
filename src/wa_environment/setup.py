from __future__ import annotations

from pathlib import Path
from warnings import simplefilter

from setuptools import SetuptoolsDeprecationWarning, find_packages, setup

# Suppress the deprecation warning for setuptools,
# there's no alternative as this is the tooling for ROS2 Humble still
simplefilter("ignore", category=SetuptoolsDeprecationWarning)


def recursive_data(prefix: Path, source: Path) -> list[tuple[str, list[str]]]:
    data = [str(path) for path in source.iterdir() if not path.is_dir()]
    subdata = [
        sample
        for path in source.iterdir()
        if path.is_dir()
        for sample in recursive_data(prefix / path.name, path)
    ]
    if len(data) == 0:
        return subdata
    return [(str(prefix), data), *subdata]


package_name = "wa_environment"
source_dir = Path(__file__).parent

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        *recursive_data(
            Path(f"share/{package_name}/models"),
            source_dir / "models",
        ),
        *recursive_data(
            Path(f"share/{package_name}/worlds"),
            source_dir / "worlds",
        ),
        *[
            (
                f"share/{package_name}/{folder}",
                [str(path) for path in (source_dir / folder).iterdir()],
            )
            for folder in ("launch", "maps", "params", "robots", "rviz")
        ],
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Eduardo Farinati Leite",
    maintainer_email="eduardo.farinati@ufrgs.br",
    description=(
        "Warehouse model definitions for Gazebo in the warehouse automation"
        " project"
    ),
    license="MIT",
    entry_points={
        "console_scripts": [],
    },
)
