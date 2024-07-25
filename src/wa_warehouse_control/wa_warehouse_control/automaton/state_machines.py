#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path

from statemachine import State, StateMachine


class MachineConveyorBelt(StateMachine):
    empty = State(initial=True)
    full = State()

    box_in = empty.to(full)
    box_out = full.to(empty)


class MachineStorageUnit(StateMachine):
    empty = State(initial=True)
    full = State()

    box_in = empty.to(full)
    box_out = full.to(empty)


class MachineTaskTransmitter(StateMachine):
    idle = State(initial=True)
    transmitting = State()
    answering_request = State()

    task_received = idle.to(transmitting) | transmitting.to(transmitting)
    task_queue_empty = transmitting.to(idle)
    request_received = transmitting.to(answering_request)
    accept_response = answering_request.to(transmitting)
    deny_response = answering_request.to(transmitting)


class MachineRobotOnOff(StateMachine):
    off = State(initial=True)
    on = State()

    turn_on = off.to(on)
    request_task = on.to(on)
    pick_box = on.to(on)
    place_box = on.to(on)
    turn_off = on.to(off)


class MachineRobotAddRobot(StateMachine):
    main = State(initial=True)

    add_robot = main.to(main)
    remove_robot = main.to(main)


class MachineRobotGoalSensor(StateMachine):
    unset = State(initial=True)
    set_ = State()

    set_goal = unset.to(set_)
    reach_goal = set_.to(unset)


class MachineRobotTaskRequest(StateMachine):
    unset = State(initial=True)
    request = State()
    set_ = State()

    request_task = unset.to(request)
    task_rejected = request.to(unset)
    task_accepted = request.to(set_)
    task_completed = set_.to(unset)


class MachineRobotPickPlace(StateMachine):
    pick = State(initial=True)
    place = State()

    pick_box = pick.to(place)
    place_box = place.to(pick)


# ---- Restrictions ----


class RestrictionRobotWork(StateMachine):
    idle = State(initial=True)
    pick = State()
    place = State()

    accept_response = idle.to(pick)
    deny_response = idle.to(idle)
    pick_box = pick.to(place)
    place_box = place.to(idle)


class RestrictionRobotTransport(StateMachine):
    idle = State(initial=True)
    pick = State()
    transport = State()
    place = State()

    goal_reached = idle.to(pick) | transport.to(place)
    pick_box = pick.to(transport)
    place_box = place.to(idle)


class RestrictionRobotTask(StateMachine):
    no_task = State(initial=True)
    task_active = State()
    complete_task = State()

    task_accepted = no_task.to(task_active)
    place_box = task_active.to(complete_task)
    task_completed = complete_task.to(no_task)


# class RestrictionRobotBox(StateMachine):
#     no_box = State()
#     with_box = State()
#     synched = State()
#     to_pick = State(initial=True)
#     to_place = State()
#
#     box_in = no_box.to(with_box)
#     sync = with_box.to(synched)
#     pick_box = to_pick.to(to_place)
#     place_box = to_place.to(to_pick)


def draw_state_machine(path: Path, machine: StateMachine) -> None:
    machine._graph().write_png(
        str(path / f"{type(machine).__name__}.png"),
    )


def clear_state_machines(path: Path) -> None:
    for file in path.glob("*.png"):
        file.unlink()


def draw_state_machines(path: Path) -> None:
    from inspect import isclass  # noqa: PLC0415

    if not path.exists():
        path.mkdir()
    else:
        clear_state_machines(path)

    for object_ in globals().values():
        if (
            isclass(object_)
            and issubclass(object_, StateMachine)
            and object_ is not StateMachine
        ):
            draw_state_machine(path, object_())


if __name__ == "__main__":
    path = Path(__file__).parent / "renders"
    draw_state_machines(path)
