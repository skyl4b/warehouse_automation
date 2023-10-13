from __future__ import annotations

from dataclasses import dataclass, field
from threading import Lock


@dataclass
class IDHandler:
    _id_lock: Lock = field(init=False, default_factory=Lock)
    _ids: list[bool] = field(init=False, default_factory=list)

    def gen_id(self) -> int:
        """Generate a new id that may be active or not."""
        with self._id_lock:
            self._ids.append(True)
            return len(self._ids)

    @property
    def active_ids(self) -> tuple[int, ...]:
        """Get the list of active ids."""
        with self._id_lock:
            return tuple(i + 1 for i, active in enumerate(self._ids) if active)

    def _set_id_state(self, id_: int, state: bool) -> None:
        """Internal function to set an id's state."""
        with self._id_lock:
            if id_ > len(self._ids):
                raise ValueError(f"ID '{id_}' was not generated")
            # ID's start at 1 and indexing at 0
            self._ids[id_ - 1] = state

    def activate_id(self, id_: int) -> None:
        """Set a generated id as active."""
        self._set_id_state(id_, True)

    def deactivate_id(self, id_: int) -> None:
        """Set an generated id as inactive."""
        self._set_id_state(id_, False)
