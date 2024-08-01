#!/usr/bin/env python3
import xml.etree.ElementTree as ET  # noqa: N817
from pathlib import Path


def main(path: Path) -> None:
    """Parse automata from a destool project."""
    # Parse the project as a XML file
    root = ET.parse(path)

    # Find all sets of states
    states = [
        {
            state.get("id"): state.get("name")
            for state in state_set.findall("State")
        }
        for state_set in root.findall(".//StateSet")
    ]

    # Find all transitions
    transitions = []

    for model_states, model_transtitions in zip(
        states,
        root.findall(".//TransitionRelation"),
        strict=True,
    ):
        # Process the Transition elements within the current TransitionRelation
        transition_dict = {}
        for transition in model_transtitions.findall("Transition"):
            x1 = transition.get("x1")
            event = transition.get("event")
            x2 = transition.get("x2")
            transition_dict.setdefault(model_states[x1], {})[event] = (
                model_states[x2]
            )
        transitions.append(transition_dict)

    # Print the results
    for i, transition_dict in enumerate(transitions):
        print(f"StateSet {i + 1}:")
        print("Transitions:", transition_dict)
        print()


if __name__ == "__main__":
    main(Path(__file__).parents[4] / "models" / "warehouse_automation.pro")
