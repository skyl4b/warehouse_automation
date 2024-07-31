from __future__ import annotations

from typing import Final, TypedDict

STORAGE_SPACE: Final[int] = 10
"""Maximum storage space of the warehouse."""

ROBOTS: Final[int] = 5
"""Maximum number of robots in the warehouse."""


class AutomatonModel(TypedDict):
    """The automaton model of a system."""

    initial_state: str
    transitions: dict[str, dict[str, str]]


TASK_TRANSMITER_MODEL: Final[AutomatonModel] = {
    "initial_state": "Idle",
    "transitions": {
        "Idle": {f"tin_{k}": "Broadcast" for k in range(STORAGE_SPACE)}
        | {f"tout_{k}": "Broadcast" for k in range(STORAGE_SPACE)}
        | {f"request_{i}": "Reject" for i in range(ROBOTS)},
        "Broadcast": {"broadcast": "Broadcast"}
        | {f"request_{i}": "Answer" for i in range(ROBOTS)},
        "Answer": {f"accept_{i}": "Idle" for i in range(ROBOTS)}
        | {f"deny_{i}": "Idle" for i in range(ROBOTS)},
        "Reject": {f"deny_{i}": "Idle" for i in range(ROBOTS)},
    },
}
"""The automaton model of the task transmitter."""


def get_robot_model(i: int) -> AutomatonModel:
    """Get the ith robot automaton model."""
    return {
        "initial_state": "Idle",
        "transitions": {
            "Idle": {f"work_{i}": "ToStart"},
            "ToStart": {f"goal_reached_{i}": "PickBox"},
            "PickBox": {f"pick_box_{i}": "ToEnd"},
            "ToEnd": {f"goal_reached_{i}": "DropBox"},
            "DropBox": {f"drop_box_{i}": "Idle"},
        },
    }


STORAGE_SUPERVISOR: Final[AutomatonModel] = {
    "initial_state": "Idle",
    "transitions": {
        "Idle": {f"tin_{k}": "Broadcast" for k in range(STORAGE_SPACE)}
        | {f"tout_{k}": "Broadcast" for k in range(STORAGE_SPACE)}
        | {f"request_{i}": "Reject" for i in range(ROBOTS)},
        "Broadcast": {"broadcast": "Broadcast"}
        | {f"request_{i}": "Answer" for i in range(ROBOTS)},
        "Answer": {f"accept_{i}": "Idle" for i in range(ROBOTS)}
        | {f"deny_{i}": "Idle" for i in range(ROBOTS)},
        "Reject": {f"deny_{i}": "Idle" for i in range(ROBOTS)},
    },
}
"""The supervisor of the storage unit (acts over task_transmitter)."""

REQUEST_SUPERVISOR: Final[AutomatonModel] = {
    "initial_state": "Idle",
    "transitions": {},
}
"""The supervisor for robot requests of tasks."""
