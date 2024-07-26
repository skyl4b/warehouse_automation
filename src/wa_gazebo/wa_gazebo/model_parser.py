"""Parse the models from the warehouse automation project."""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

environment_dir = Path(get_package_share_directory("wa_environment"))
"""Environment description for the warehouse automation project."""

models_dir = environment_dir / "models"
"""The directory of model descriptions of the warehouse automation project."""


@dataclass
class ModelParser:
    pattern: re.Pattern[str] = field(init=False)
    """Pattern to match one of the models in model_descriptions."""

    descriptions: dict[str, str] = field(init=False)
    """Dictionary with model descriptions.

    Keys are model names and values are SDF contents.
    """

    def __post_init__(self) -> None:
        """Load models and set model pattern."""
        self.descriptions = {
            file.parent.name: file.read_text()
            for file in models_dir.glob("*/model.sdf")
        }

        # Calculate model pattern from descriptions
        joined_models = "|".join(
            re.escape(model) for model in self.descriptions
        )
        self.pattern = re.compile(f"^({joined_models})")

    def parse(self, name: str) -> str | None:
        """Parse model description from its name."""
        if (match_ := self.pattern.match(name)) is None:
            return None
        return match_.group(0)
