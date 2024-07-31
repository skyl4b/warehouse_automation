from __future__ import annotations

from dataclasses import dataclass, field
from typing import TypeAlias

AutomatonTransitions: TypeAlias = dict[str, dict[str, str]]
"""Automaton transitions described as mapping.

The mapping consists of a state->event->next state sequence
representes as two chained Python dictionaries.
"""


@dataclass
class Automaton:
    """A finite state automaton (FSM) with events that can be disabled."""

    state: str
    """The current state of the automaton."""

    transitions: AutomatonTransitions
    """The transition function of the automaton"""

    states: set[str] = field(init=False)
    """All states of the automaton."""

    events: set[str] = field(init=False)
    """All events of the automaton."""

    enabled_events: set[str] = field(init=False)
    """Enabled events of the automaton"""

    def __post_init__(self) -> None:
        """Initialize events and states."""
        self.states = set(self.transitions.keys())
        self.events = {
            event
            for state_transitions in self.transitions.values()
            for event in state_transitions
        }
        self.reset_enabled_events()

    def current_transitions(self) -> dict[str, str]:
        """Get the transitions the automaton may undergo in this state."""
        return self.transitions.get(self.state, {})

    def current_events(self) -> set[str]:
        """Get the events that may occur in this state."""
        return set(self.current_transitions().keys())

    def transition(self, event: str) -> None:
        """Transition between states."""
        if event not in self.events:
            # Event ignored by the automaton
            return

        if (next_state := self.current_transitions().get(event)) is None:
            raise ValueError(
                f"Event '{event}' is not possible in state '{self.state}'",
            )

        if event not in self.enabled_events:
            raise ValueError(f"Event '{event}' is disabled")

        self.state = next_state
        self.enabled_events = self.events

    def restrict_enabled_events(self, events: set[str]) -> None:
        """Restrict currently enabled events to a subset.."""
        self.enabled_events = self.enabled_events.intersection(events)

    def reset_enabled_events(self) -> None:
        """Reset all events to enabled."""
        self.enabled_events = self.events
