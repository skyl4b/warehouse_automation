"""Type definitions for the map of the warehouse automation project."""

from __future__ import annotations

from typing import Final, TypedDict


class Position(TypedDict):
    """A position in 3D space."""

    x: float
    y: float
    z: float


class Model(TypedDict):
    """A model in 3D space."""

    name: str
    position: Position


class ConveyorBelts(TypedDict):
    """A conveyor belt in the warehouse automation project."""

    input: list[Model]
    output: list[Model]


class StorageUnit(Model):
    """A storage unit in the warehouse automation project."""

    empty: bool


class Map(TypedDict):
    """The map of a simulation in the warehouse automation project."""

    conveyor_belts: ConveyorBelts
    storage_units: list[StorageUnit]


BASE_MAP: Final[Map] = {
    "conveyor_belts": {
        "input": [
            {
                "name": "conveyor_belt_0_0",
                "position": {"x": 3.0, "y": -8.7, "z": 0.4},
            },
            {
                "name": "conveyor_belt_0_1",
                "position": {"x": 0.0, "y": -8.7, "z": 0.4},
            },
            {
                "name": "conveyor_belt_0_2",
                "position": {"x": -3.0, "y": -8.7, "z": 0.4},
            },
        ],
        "output": [
            {
                "name": "conveyor_belt_1_0",
                "position": {"x": 3.0, "y": 8.7, "z": 0.4},
            },
            {
                "name": "conveyor_belt_1_1",
                "position": {"x": 0.0, "y": 8.7, "z": 0.4},
            },
            {
                "name": "conveyor_belt_1_2",
                "position": {"x": -3.0, "y": 8.7, "z": 0.4},
            },
        ],
    },
    "storage_units": [
        {
            "name": "storage_unit_0_0",
            "position": {"x": 1.5, "y": -6.0, "z": 0.28},
            "empty": True,
        },
        {
            "name": "storage_unit_1_0",
            "position": {"x": 1.5, "y": -3.0, "z": 0.28},
            "empty": True,
        },
        {
            "name": "storage_unit_2_0",
            "position": {"x": 1.5, "y": 0.0, "z": 0.28},
            "empty": True,
        },
        {
            "name": "storage_unit_3_0",
            "position": {"x": 1.5, "y": 3.0, "z": 0.28},
            "empty": True,
        },
        {
            "name": "storage_unit_4_0",
            "position": {"x": 1.5, "y": 6.0, "z": 0.28},
            "empty": True,
        },
        {
            "name": "storage_unit_0_1",
            "position": {"x": -1.5, "y": -6.0, "z": 0.28},
            "empty": True,
        },
        {
            "name": "storage_unit_1_1",
            "position": {"x": -1.5, "y": -3.0, "z": 0.28},
            "empty": True,
        },
        {
            "name": "storage_unit_2_1",
            "position": {"x": -1.5, "y": 0.0, "z": 0.28},
            "empty": True,
        },
        {
            "name": "storage_unit_3_1",
            "position": {"x": -1.5, "y": 3.0, "z": 0.28},
            "empty": True,
        },
        {
            "name": "storage_unit_4_1",
            "position": {"x": -1.5, "y": 6.0, "z": 0.28},
            "empty": True,
        },
    ],
}
"""The base map expected from the warehouse automation project."""
