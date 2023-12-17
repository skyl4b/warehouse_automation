from __future__ import annotations

from abc import ABC
from dataclasses import dataclass, field
from threading import Lock
from typing import ClassVar


@dataclass
class UIDHandler:
    _uid_lock: Lock = field(init=False, default_factory=Lock)
    _uids: list[bool] = field(init=False, default_factory=list)

    def gen_uid(self) -> int:
        """Generate a new uid that may be active or not."""
        with self._uid_lock:
            self._uids.append(True)
            return len(self._uids)

    @property
    def active_uids(self) -> tuple[int, ...]:
        """Get the list of active uids."""
        with self._uid_lock:
            return tuple(
                i + 1 for i, active in enumerate(self._uids) if active
            )

    def _set_uid_state(self, uid: int, state: bool) -> None:
        """Set an uid's state."""
        with self._uid_lock:
            if uid > len(self._uids) or uid <= 0:
                raise ValueError(f"ID '{uid}' was not generated")
            # IDs start at 1 and indexing starts at 0
            self._uids[uid - 1] = state

    def activate_uid(self, uid: int) -> None:
        """Set a generated uid as active."""
        self._set_uid_state(uid, True)

    def deactivate_uid(self, uid: int) -> None:
        """Set an generated uid as inactive."""
        self._set_uid_state(uid, False)


class UIDIndexed(ABC):
    """Abstract class for objects that have a unique id."""

    _uid_handler: ClassVar[UIDHandler]
    _uid_handler_lock: ClassVar[Lock] = Lock()

    def __init__(self) -> None:
        """Initialize this object with an uid."""
        with self._uid_handler_lock:
            if not hasattr(type(self), "_uid_handler"):
                type(self)._uid_handler = UIDHandler()

        self._uid = self._uid_handler.gen_uid()

    @property
    def uid(self) -> int:
        """Get this object's uid."""
        return self._uid

    @classmethod
    def get_active_uids(cls) -> tuple[int, ...]:
        """Get the list of current active ids."""
        with cls._uid_handler_lock:
            if hasattr(cls, "_uid_handler"):
                return cls._uid_handler.active_uids
        return ()

    def activate(self) -> None:
        """Activate this object."""
        self._uid_handler.activate_uid(self._uid)

    def deactivate(self) -> None:
        """Deactivate this object."""
        self._uid_handler.deactivate_uid(self._uid)
