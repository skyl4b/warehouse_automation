#!/usr/bin/env python3
"""Utilities to create visualizations for the warehouse automation project."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, cast

import matplotlib.dates as mdates
import matplotlib.pyplot as plt
import numpy as np
from bag_display import parse_bag
from cycler import cycler
from wa_interfaces import msg as wa_msgs

if TYPE_CHECKING:
    from datetime import datetime


def plot_simulation() -> None:
    """Plot simulation results from the warehouse automation project."""
    configure_matplotlib_for_scientific_plots()

    # Parse bag recording
    bag = parse_bag(Path(__file__).parent / "bag_recordings" / "demand")

    # Demand plot
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
    )


def plot_demand(
    demand: list[tuple[datetime, wa_msgs.Demand]],
    unbounded_demand: list[tuple[datetime, wa_msgs.Demand]],
    time_unit: str = "seconds",  # 'seconds', 'minutes', 'hours', etc.
) -> None:
    """Plot the demand of the warehouse during the simulation."""
    # Time
    t_d = [time for time, _ in demand]
    t_u = [time for time, _ in unbounded_demand]

    # Demand
    d_i = [message.input_demand for _, message in demand]
    d_o = [message.output_demand for _, message in demand]

    # Unbounded demand
    u_i = [message.input_demand for _, message in unbounded_demand]
    u_o = [message.output_demand for _, message in unbounded_demand]

    # Convert to relative time
    s_d = t_d[0]
    t_d = [(t_i - s_d).total_seconds() for t_i in t_d]
    s_u = t_u[0]
    t_u = [(t_i - s_u).total_seconds() for t_i in t_u]

    # Plot
    _, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

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
    ax1.set_xlabel(f"Time [{time_unit}]")
    ax1.set_ylabel(r"Input demand [box units]")
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
    ax2.set_xlabel(f"Time [{time_unit}]")
    ax2.set_ylabel(r"Output demand [box units]")
    ax2.set_ylim((-0.25, max(*u_i, *u_o) + 1))
    ax2.set_yticks(list(range(max(*u_i, *u_o) + 1)))
    ax2.legend()

    plt.tight_layout(pad=5.0)
    plt.show()


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


def accumulate_positive(values: list[int | float]) -> list[int | float]:
    """Accumulate positive changes in an array (ignoring negatives)."""
    acc = [values[0]]
    for i in range(len(values) - 1):
        delta = values[i + 1] - values[i]
        acc.append(acc[i] + max(delta, 0))
    return acc


plot_simulation()
