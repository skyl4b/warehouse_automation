from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class Automaton:
    """A finite state automaton (FSM)."""

    state: str
    """The current state of the automaton."""

    transitions: dict[str, dict[str, str]]
    """The transition function of the automaton"""

    states: set[str] = field(init=False)
    """All states of the automaton."""

    events: set[str] = field(init=False)
    """All events of the automaton."""

    enabled_events: set[str] = field(init=False)
    """Enabled event of the automaton"""

    def __post_init__(self) -> None:
        """Initialize events and states."""
        self.states = set(self.transitions.keys())
        self.events = {
            event
            for state_transitions in self.transitions.values()
            for event in state_transitions
        }
        # All events start enabled
        self.enabled_events = self.events

    def current_transitions(self) -> dict[str, str]:
        """Get the transitions the automaton may undergo in this state."""
        return self.transitions.get(self.state, {})

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

    def disable_event(self, event: str) -> None:
        """Disable an event from occuring."""
        if event in self.enabled_events:
            self.enabled_events.remove(event)

    def enable_event(self, event: str) -> None:
        """Enable an event, it may occur."""
        if event in self.events:
            self.enabled_events.add(event)
