#!/usr/bin/env python3
"""Utilities to create visualizations for the warehouse automation project."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import TYPE_CHECKING, Final, cast

import matplotlib.pyplot as plt
from bag_display import parse_bag
from cycler import cycler
from std_msgs import msg as std_msgs
from wa_interfaces import msg as wa_msgs

if TYPE_CHECKING:
    from datetime import datetime

TIME_UNITS: Final[dict[str, float]] = {
    "seconds": 1.0,
    "minutes": 60.0,
    "hours": 60.0 * 60.0,
}
"""Time unit from seconds conversion."""


def main(
    path: Path, time_unit: str = "minutes", save_plots: bool = False
) -> None:
    """Plot simulation results from the warehouse automation project."""
    configure_matplotlib_for_scientific_plots()

    # Parse bag recording
    bag = parse_bag(path)

    # Demand
    plot_demand(
        [
            (time, cast(wa_msgs.Demand, demand))
            for topic, demand, time in bag
            if topic == "/wa/demand_generator/demand"
        ],
        [
            (time, cast(wa_msgs.Demand, demand))
            for topic, demand, time in bag
            if topic == "/wa/demand_generator/unbounded_demand"
        ],
        [
            (time, cast(std_msgs.String, map_))
            for topic, map_, time in bag
            if topic == "/wa/task_transmitter/map"
        ],
        unit=time_unit,
        save_plot=save_plots,
    )

    # Task stats
    plot_task_stats(
        [
            (time, cast(std_msgs.UInt8, task))
            for topic, task, time in bag
            if topic == "/wa/task/started"
        ],
        [
            (time, cast(std_msgs.UInt8, task))
            for topic, task, time in bag
            if topic == "/wa/task/completed"
        ],
        unit=time_unit,
        save_plot=save_plots,
    )


def plot_demand(
    demand: list[tuple[datetime, wa_msgs.Demand]],
    unbounded_demand: list[tuple[datetime, wa_msgs.Demand]],
    map_: list[tuple[datetime, std_msgs.String]],
    unit: str = "minutes",
    save_plot: bool = False,
) -> None:
    """Plot the demand of the warehouse during the simulation."""
    # Time
    t_d = [time for time, _ in demand]
    t_u = [time for time, _ in unbounded_demand]
    t_s = [time for time, _ in map_]

    # Demand
    d_i = [message.input_demand for _, message in demand]
    d_o = [message.output_demand for _, message in demand]

    # Unbounded demand
    u_i = [message.input_demand for _, message in unbounded_demand]
    u_o = [message.output_demand for _, message in unbounded_demand]

    # Storage
    s = [
        sum(su != "E" for su in message.data.split("||", maxsplit=3)[1])
        for _, message in map_
    ]

    # Convert to relative time
    s_d = t_d[0]
    t_d = [(t_i - s_d).total_seconds() / TIME_UNITS[unit] for t_i in t_d]
    s_u = t_u[0]
    t_u = [(t_i - s_u).total_seconds() / TIME_UNITS[unit] for t_i in t_u]
    s_s = t_s[0]
    t_s = [(t_i - s_s).total_seconds() / TIME_UNITS[unit] for t_i in t_s]

    # Plot
    _, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(12, 6))

    # Zeroth order interpolation plot for input demand
    ax1.plot(t_d, d_i, label=r"$D_i(t)$", drawstyle="steps-post", color="k")
    ax1.plot(
        t_u,
        u_i,
        label=r"$U_i(t)$",
        drawstyle="steps-post",
        linestyle="--",
        color="darkblue",
    )
    ax1.set_xlabel(f"Time [{unit}]")
    ax1.set_ylabel(r"Input demand [box units]")
    ax1.set_xlim((0, 10))
    ax1.set_ylim((-0.25, max(*u_i, *u_o) + 1))
    ax1.set_yticks(list(range(max(*u_i, *u_o) + 1)))
    ax1.legend()

    # Zeroth order interpolation plot for output demand
    ax2.plot(t_d, d_o, label=r"$D_o(t)$", drawstyle="steps-post", color="k")
    ax2.plot(
        t_u,
        u_o,
        label=r"$U_o(t)$",
        drawstyle="steps-post",
        linestyle="--",
        color="darkblue",
    )
    ax2.set_xlabel(f"Time [{unit}]")
    ax2.set_ylabel(r"Output demand [box units]")
    ax2.set_xlim((0, 10))
    ax2.set_ylim((-0.25, max(*u_i, *u_o) + 1))
    ax2.set_yticks(list(range(max(*u_i, *u_o) + 1)))
    ax2.legend()

    # Storage plot
    ax3.plot(t_s, s, label=r"$S(t)$", drawstyle="steps-post", color="k")
    ax3.set_xlabel(f"Time [{unit}]")
    ax3.set_ylabel(r"Warehouse storage [box units]")
    ax3.set_xlim((0, 10))
    ax3.set_ylim((-0.25, 10 + 0.25))
    ax3.set_yticks(list(range(10 + 1)))
    ax3.legend()

    plt.tight_layout(pad=5.0)
    if save_plot:
        plt.savefig("demand_plot.pdf")
    else:
        plt.show()


def plot_task_stats(
    task_started: list[tuple[datetime, std_msgs.UInt8]],
    task_completed: list[tuple[datetime, std_msgs.UInt8]],
    unit: str = "minutes",
    save_plot: bool = False,
) -> None:
    """Plot the stats of tasks during the simulation."""
    # Task times
    s_t = {task.data: time for time, task in task_started}
    c_t = {task.data: time for time, task in task_completed}

    # Task duration data
    # Time
    t_d = [s_t[task] for task in s_t if task in c_t]

    # Convert to relative time
    s_c = min(t_d)
    t_d = [(t_i - s_c).total_seconds() for t_i in t_d]

    # Task duration
    d_t = [
        (c_t[task] - s_t[task]).total_seconds() for task in s_t if task in c_t
    ]
    a_d_t = rolling_average(t_d, d_t, w=60.0)

    # Parallel task data
    # Time
    t_p = sorted({*s_t.values(), *c_t.values()})

    # Parallel tasks
    p_t = [0 for _ in range(len(t_p))]
    for i, t in enumerate(t_p):
        for time in s_t.values():
            if time == t:
                for j in range(i, len(t_p)):
                    p_t[j] += 1
        for time in c_t.values():
            if time == t:
                for j in range(i, len(t_p)):
                    p_t[j] -= 1

    # Convert to relative time
    s_p = min(t_p)
    t_p = [(t_i - s_p).total_seconds() for t_i in t_p]

    # Convert units
    t_d = [t_i / TIME_UNITS[unit] for t_i in t_d]
    a_d_t = [a_d_i / TIME_UNITS[unit] for a_d_i in a_d_t]
    t_p = [t_i / TIME_UNITS[unit] for t_i in t_p]

    # Plot
    _, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

    # Zeroth order interpolation plot for task duration
    ax1.plot(
        t_d,
        a_d_t,
        label=r"$T^{\text{Average}}_d(t)$",
        drawstyle="steps-post",
        color="k",
    )
    ax1.set_xlabel(f"Time [{unit}]")
    ax1.set_ylabel(rf"Average task duration [{unit}]")
    ax1.set_xlim((0, 10))
    ax1.legend()

    # Zeroth order interpolation plot for number of parallel tasks
    ax2.plot(t_p, p_t, label=r"$T_p(t)$", drawstyle="steps-post", color="k")
    ax2.set_xlabel(f"Time [{unit}]")
    ax2.set_ylabel(r"Number of parallel tasks [task units]")
    ax2.set_xlim((0, 10))
    ax2.set_ylim((-0.25, 4 + 0.25))
    ax2.set_yticks(list(range(4 + 1)))
    ax2.legend()

    plt.tight_layout(pad=5.0)
    if save_plot:
        plt.savefig("task_stats.pdf")
    else:
        plt.show()


def rolling_average(t: list[float], x: list[float], w: float) -> list[float]:
    """Compute the rolling average of the last w seconds of data."""
    r_avr = []
    for i in range(len(t)):
        start = t[i] - w

        # data points within the window
        window_data = [j for j in range(i, -1, -1) if t[j] >= start]

        # Window average
        if window_data:
            avr = sum(x[j] for j in window_data) / len(window_data)
        else:
            avr = 0

        r_avr.append(avr)

    return r_avr


def configure_matplotlib_for_scientific_plots() -> None:
    """Configure Matplotlib to look nicer in a paper."""
    # Based on: https://github.com/garrettj403/SciencePlots/blob/master/scienceplots/styles/science.mplstyle
    # License: MIT
    plt.rcParams.update({
        "axes.prop_cycle": cycler(
            "color",
            [
                "#0C5DA5",
                "#00B945",
                "#FF9500",
                "#FF2C00",
                "#845B97",
                "#474747",
                "#9e9e9e",
            ],
        ),
        "figure.figsize": [3.5, 2.625],
        "xtick.direction": "in",
        "xtick.major.size": 3,
        "xtick.major.width": 0.5,
        "xtick.minor.size": 1.5,
        "xtick.minor.width": 0.5,
        "xtick.minor.visible": True,
        "xtick.top": True,
        "ytick.direction": "in",
        "ytick.major.size": 3,
        "ytick.major.width": 0.5,
        "ytick.minor.size": 1.5,
        "ytick.minor.width": 0.5,
        "ytick.minor.visible": True,
        "ytick.right": True,
        "axes.linewidth": 0.5,
        "grid.linewidth": 0.5,
        "lines.linewidth": 1.0,
        "legend.frameon": False,
        "savefig.bbox": "tight",
        "savefig.pad_inches": 0.05,
        "font.family": "serif",
        "mathtext.fontset": "dejavuserif",
        "text.usetex": False,
        "text.latex.preamble": r"\usepackage{amsmath} \usepackage{amssymb}",
    })


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description="""Plot the warehouse automation simulation results.""",
    )

    parser.add_argument(
        "path",
        type=Path,
        help="path of the ros2 bag simulation recording",
    )
    parser.add_argument(
        "-u",
        "--time-unit",
        default="minutes",
        help="Time units for the plots",
    )
    parser.add_argument(
        "-s",
        "--save-plots",
        action="store_true",
        help="Save plots to disk.",
    )

    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    main(
        args.path,
        args.time_unit,
        args.save_plots,
    )
