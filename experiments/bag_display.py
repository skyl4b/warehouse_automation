#!/usr/bin/env python3
"""Display a ros2 bag recording on the terminal."""

from __future__ import annotations

import argparse
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import cast

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


def main(path: Path) -> None:
    """Display a bag recording on the terminal."""
    bag = parse_bag(path)
    relative_to = None
    for topic, message, timestamp in bag:
        if relative_to is None:
            relative_to = timestamp
        print(
            f"{timestamp - relative_to}: Topic: {topic}, Message: {message}",
        )


def parse_bag(path: Path) -> list[tuple[str, object, datetime]]:
    """Extract the data from a ros2 bag."""
    reader = SequentialReader()

    # Open bag
    reader.open(
        StorageOptions(
            uri=str(path),
            storage_id="sqlite3",
        ),
        ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    # Load the mapping of topics to types
    topic_type_map = get_topic_types(reader)

    # Parse data
    bag = []
    while reader.has_next():
        topic, data, timestamp = cast(
            tuple[str, bytes, int],
            reader.read_next(),
        )
        message = deserialize_message(data, topic_type_map[topic])
        bag.append((topic, message, convert_timestamp(timestamp)))
    return bag


def get_topic_types(reader: SequentialReader) -> dict[str, object]:
    """Get a dictionary of topic names to their types."""
    return {
        topic.name: get_message(topic.type)
        for topic in reader.get_all_topics_and_types()
    }


def convert_timestamp(timestamp: int) -> datetime:
    """Convert the unix-time timestamp into a datetime."""
    seconds = timestamp // 1_000_000_000
    nanosec = timestamp % 1_000_000_000
    return datetime.fromtimestamp(seconds, tz=timezone.utc) + timedelta(
        microseconds=nanosec / 1_000,
    )


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description="""Display a bag recording on the terminal.""",
    )

    parser.add_argument(
        "path",
        type=Path,
        help="path of the ros2 bag recording",
    )

    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    main(args.path)
