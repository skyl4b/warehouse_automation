"""Utilities to create visualizations for the warehouse automation project."""

from __future__ import annotations

from pathlib import Path

import matplotlib.dates as mdates
import matplotlib.pyplot as plt
import numpy as np
from bag_display import parse_bag
from cycler import cycler


def plot_simulation() -> None:
    """Plot simulation results from the warehouse automation project."""
    configure_matplotlib_for_scientific_plots()

    # Sample data
    bag = parse_bag(Path(__file__).parent / "recordings" / "demand")
    t = [item[2] for item in bag]

    # Demand
    d_i = [item[1].input_demand for item in bag]
    d_o = [item[1].output_demand for item in bag]

    # Unbounded demand
    u_i = accumulate_positive(d_i)
    u_o = accumulate_positive(d_o)

    # Plot
    plt.figure()

    # Relative datetimes
    start = mdates.date2num(t[0])
    t = [mdates.date2num(t_i) - start for t_i in t]

    # Zeroth order interpolation plot
    plt.plot(t, d_i, label=r"$D_i(t)$", drawstyle="steps-post")
    plt.plot(t, d_o, label=r"$D_o(t)$", drawstyle="steps-post")
    plt.plot(t, u_i, label=r"$U_i(t)$", drawstyle="steps-post", linestyle="--")
    plt.plot(t, u_o, label=r"$U_o(t)$", drawstyle="steps-post", linestyle="--")

    plt.title("Demand plot")
    plt.xlabel(r"$t$")
    plt.ylabel(r"$D(t)$")
    plt.ylim((-1, max(*u_i, *u_o) + 1))
    plt.yticks(list(range(max(*u_i, *u_o) + 1)))

    plt.legend()
    plt.tight_layout()
    # plt.savefig("example_plot.pdf")
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
