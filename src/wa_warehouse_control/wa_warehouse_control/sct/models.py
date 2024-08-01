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
        | {f"deny_{i}": "Broadcast" for i in range(ROBOTS)},
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
    "initial_state": "Idle|Empty",
    "transitions": {
        "Idle|Empty": {
            f"tin_{k}": "Broadcast|Full" for k in range(STORAGE_SPACE)
        }
        | {f"request_{i}": "Reject|Empty" for i in range(ROBOTS)},
        "Broadcast|Full": {"broadcast": "Broadcast|Full"}
        | {f"request_{i}": "Answer|Full" for i in range(ROBOTS)},
        "Answer|Full": {f"accept_{i}": "Idle|Full" for i in range(ROBOTS)}
        | {f"deny_{i}": "Broadcast|Full" for i in range(ROBOTS)},
        "Idle|Full": {
            f"tout_{k}": "Broadcast|Empty" for k in range(STORAGE_SPACE)
        }
        | {f"request_{i}": "Reject|Full" for i in range(ROBOTS)},
        "Broadcast|Empty": {"broadcast": "Broadcast|Empty"}
        | {f"request_{i}": "Answer|Empty" for i in range(ROBOTS)},
        "Answer|Empty": {f"accept_{i}": "Idle|Empty" for i in range(ROBOTS)}
        | {f"deny_{i}": "Broadcast|Empty" for i in range(ROBOTS)},
        "Reject|Empty": {f"deny_{i}": "Idle|Empty" for i in range(ROBOTS)},
        "Reject|Full": {f"deny_{i}": "Idle|Full" for i in range(ROBOTS)},
    },
}
"""The supervisor of the storage unit (acts over task_transmitter)."""

REQUEST_SUPERVISOR: Final[AutomatonModel] = {
    "initial_state": "Idle",
    "transitions": {
        "Idle|Idle|Idle": {
            f"tin_{k}": "Broadcast|Idle|Idle" for k in range(STORAGE_SPACE)
        }
        | {f"tout_{k}": "Broadcast|Idle|Idle" for k in range(STORAGE_SPACE)},
        "Broadcast|Idle|Idle": {"broadcast": "Broadcast|Idle|Request"},
        "Broadcast|Idle|Request": {"broadcast": "Broadcast|Idle|Request"}
        | {f"request_{i}": "Answer|Idle|Wait" for i in range(ROBOTS)},
        "Answer|Idle|Wait": {
            f"accept_{i}": "Idle|Idle|Work" for i in range(ROBOTS)
        }
        | {f"deny_{i}": "Broadcast|Idle|Idle" for i in range(ROBOTS)},
        "Idle|Idle|Work": {
            f"tin_{k}": "Broadcast|Idle|Work" for k in range(STORAGE_SPACE)
        }
        | {f"tout_{k}": "Broadcast|Idle|Work" for k in range(STORAGE_SPACE)}
        | {f"work_{i}": "Idle|ToStart|WorkDone" for i in range(ROBOTS)},
        "Broadcast|Idle|Work": {"broadcast": "Broadcast|Idle|Work"}
        | {f"work_{i}": "Broadcast|ToStart|WorkDone" for i in range(ROBOTS)},
        "Idle|ToStart|WorkDone": {
            f"tin_{k}": "Broadcast|ToStart|WorkDone"
            for k in range(STORAGE_SPACE)
        }
        | {
            f"tout_{k}": "Broadcast|ToStart|WorkDone"
            for k in range(STORAGE_SPACE)
        }
        | {
            f"goal_reached_{i}": "Idle|PickBox|WorkDone" for i in range(ROBOTS)
        },
        "Broadcast|ToStart|WorkDone": {
            "broadcast": "Broadcast|ToStart|WorkDone",
        }
        | {
            f"goal_reached_{i}": "Broadcast|PickBox|WorkDone"
            for i in range(ROBOTS)
        },
        "Idle|PickBox|WorkDone": {
            f"tin_{k}": "Broadcast|PickBox|WorkDone"
            for k in range(STORAGE_SPACE)
        }
        | {
            f"tout_{k}": "Broadcast|PickBox|WorkDone"
            for k in range(STORAGE_SPACE)
        }
        | {f"pick_box_{i}": "Idle|ToEnd|WorkDone" for i in range(ROBOTS)},
        "Broadcast|PickBox|WorkDone": {
            "broadcast": "Broadcast|PickBox|WorkDone",
        }
        | {f"pick_box_{i}": "Broadcast|ToEnd|WorkDone" for i in range(ROBOTS)},
        "Idle|ToEnd|WorkDone": {
            f"tin_{k}": "Broadcast|ToEnd|WorkDone"
            for k in range(STORAGE_SPACE)
        }
        | {
            f"tout_{k}": "Broadcast|ToEnd|WorkDone"
            for k in range(STORAGE_SPACE)
        }
        | {
            f"goal_reached_{i}": "Idle|DropBox|WorkDone" for i in range(ROBOTS)
        },
        "Broadcast|ToEnd|WorkDone": {"broadcast": "Broadcast|ToEnd|WorkDone"}
        | {
            f"goal_reached_{i}": "Broadcast|DropBox|WorkDone"
            for i in range(ROBOTS)
        },
        "Idle|DropBox|WorkDone": {
            f"tin_{k}": "Broadcast|DropBox|WorkDone"
            for k in range(STORAGE_SPACE)
        }
        | {
            f"tout_{k}": "Broadcast|DropBox|WorkDone"
            for k in range(STORAGE_SPACE)
        }
        | {f"drop_box_{i}": "Idle|Idle|Idle" for i in range(ROBOTS)},
        "Broadcast|DropBox|WorkDone": {
            "broadcast": "Broadcast|DropBox|WorkDone",
        }
        | {f"drop_box_{i}": "Broadcast|Idle|Idle" for i in range(ROBOTS)},
    },
}
"""The supervisor for robot requests of tasks."""
