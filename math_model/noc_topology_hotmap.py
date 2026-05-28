#!/usr/bin/env python3

import os
os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import argparse
from pathlib import Path
import enum

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from matplotlib import colormaps, colors
from matplotlib.lines import Line2D


# ============================================================
# Global color scale
# ============================================================
# Default fixed scale: 0-32, so every topology is visually comparable.
# Use --global-vmax auto to compute a shared scale from selected topologies.

GLOBAL_VMIN = 0
GLOBAL_VMAX = 32


# ============================================================
# Common traffic model
# ============================================================

def all_nodes(n):
    return [(x, y) for y in range(n) for x in range(n)]


def greedy_2hop_steps(src_coord, dst_coord):
    """
    Greedy 2-hop movement along one dimension.

    If remaining distance >= 2, use a 2-hop express link.
    Otherwise, use a local 1-hop mesh link.

    Return:
        (1, k) means local link k <-> k+1
        (2, k) means express link k <-> k+2
    """
    cur = src_coord
    steps = []

    while cur != dst_coord:
        delta = dst_coord - cur

        if delta > 0:
            step = 2 if delta >= 2 else 1
        else:
            step = -2 if delta <= -2 else -1

        nxt = cur + step
        link_start = min(cur, nxt)

        steps.append((abs(step), link_start))
        cur = nxt

    return steps


# ============================================================
# Mesh / Ruche routing models
# ============================================================

def xy_mesh_path(src, dst):
    """
    Standard XY routing on 2D mesh.

    First X using local H1 links.
    Then Y using local V1 links.
    """
    sx, sy = src
    dx, dy = dst
    links = []

    # X phase
    for x in range(min(sx, dx), max(sx, dx)):
        links.append(("H1", sy, x))

    # Y phase
    for y in range(min(sy, dy), max(sy, dy)):
        links.append(("V1", y, dx))

    return links


def xfirst_y_ruche_path(src, dst):
    """
    2-hop Y-direction half-ruche.

    X phase:
        use local H1 mesh links only.

    Y phase:
        use greedy 2-hop Y express links when possible.
    """
    sx, sy = src
    dx, dy = dst
    links = []

    # X phase: local mesh links only
    for x in range(min(sx, dx), max(sx, dx)):
        links.append(("H1", sy, x))

    # Y phase: local V1 or express V2
    for step_len, y in greedy_2hop_steps(sy, dy):
        if step_len == 1:
            links.append(("V1", y, dx))
        else:
            links.append(("V2", y, dx))

    return links


def xfirst_full_ruche_path(src, dst):
    """
    2-hop full-ruche.

    Routing rule:
        X-first dimension-ordered routing.
        Inside X dimension, use greedy 2-hop X express links.
        Inside Y dimension, use greedy 2-hop Y express links.
    """
    sx, sy = src
    dx, dy = dst
    links = []

    # X phase: local H1 or express H2
    for step_len, x in greedy_2hop_steps(sx, dx):
        if step_len == 1:
            links.append(("H1", sy, x))
        else:
            links.append(("H2", sy, x))

    # Y phase: local V1 or express V2
    for step_len, y in greedy_2hop_steps(sy, dy):
        if step_len == 1:
            links.append(("V1", y, dx))
        else:
            links.append(("V2", y, dx))

    return links


def map_mesh_ruche_traffic(n, topology):
    """
    Enumerate all ordered source->destination accesses.
    source == destination is excluded.

    Count how many ordered accesses use each logical NoC channel.
    Direction is aggregated on the same physical/logical channel.
    """
    h1_load = np.zeros((n, n - 1), dtype=int)
    h2_load = np.zeros((n, max(n - 2, 0)), dtype=int)
    v1_load = np.zeros((n - 1, n), dtype=int)
    v2_load = np.zeros((max(n - 2, 0), n), dtype=int)

    nodes = all_nodes(n)

    for src in nodes:
        for dst in nodes:
            if src == dst:
                continue

            if topology == "mesh":
                path = xy_mesh_path(src, dst)
            elif topology == "y_ruche_2hop":
                path = xfirst_y_ruche_path(src, dst)
            elif topology == "full_ruche_2hop":
                path = xfirst_full_ruche_path(src, dst)
            else:
                raise ValueError(f"Unknown mesh/ruche topology: {topology}")

            for typ, a, b in path:
                if typ == "H1":
                    y, x = a, b
                    h1_load[y, x] += 1
                elif typ == "H2":
                    y, x = a, b
                    h2_load[y, x] += 1
                elif typ == "V1":
                    y, x = a, b
                    v1_load[y, x] += 1
                elif typ == "V2":
                    y, x = a, b
                    v2_load[y, x] += 1
                else:
                    raise ValueError(f"Unknown link type: {typ}")

    return h1_load, h2_load, v1_load, v2_load


# ============================================================
# Full-torus routing model
# ============================================================

class RouteDirection(enum.Enum):
    Eject = 0
    North = 1
    East = 2
    South = 3
    West = 4


def torus_next_hop(src, dst, n):
    """
    Same routing rule as gen_routing_table.py:

    1. X dimension first.
    2. Use the shorter torus direction.
    3. If two directions tie, even coordinate chooses positive direction
       and odd coordinate chooses negative direction.
    4. Then route Y dimension using the same logic.
    """
    src_x, src_y = src
    dst_x, dst_y = dst

    if src_x != dst_x:
        east_dist = (dst_x - src_x) % n
        west_dist = (src_x - dst_x) % n

        if east_dist < west_dist:
            return RouteDirection.East
        elif west_dist < east_dist:
            return RouteDirection.West
        elif src_x % 2 == 0:
            return RouteDirection.East
        else:
            return RouteDirection.West

    if src_y != dst_y:
        north_dist = (dst_y - src_y) % n
        south_dist = (src_y - dst_y) % n

        if north_dist < south_dist:
            return RouteDirection.North
        elif south_dist < north_dist:
            return RouteDirection.South
        elif src_y % 2 == 0:
            return RouteDirection.North
        else:
            return RouteDirection.South

    return RouteDirection.Eject


def torus_advance(node, direction, n):
    x, y = node

    if direction == RouteDirection.East:
        return ((x + 1) % n, y)
    if direction == RouteDirection.West:
        return ((x - 1) % n, y)
    if direction == RouteDirection.North:
        return (x, (y + 1) % n)
    if direction == RouteDirection.South:
        return (x, (y - 1) % n)
    if direction == RouteDirection.Eject:
        return node

    raise ValueError(f"Unknown torus direction: {direction}")


def torus_route_path(src, dst, n):
    """
    Return complete directed torus path:
        [((x0,y0), (x1,y1), direction), ...]
    """
    if src == dst:
        return []

    cur = src
    path = []
    visited = set()

    for _ in range(n * n + 1):
        state = (cur, dst)
        if state in visited:
            raise RuntimeError(
                f"Torus routing loop detected: src={src}, dst={dst}, cur={cur}, path={path}"
            )
        visited.add(state)

        direction = torus_next_hop(cur, dst, n)
        if direction == RouteDirection.Eject:
            break

        nxt = torus_advance(cur, direction, n)
        path.append((cur, nxt, direction))
        cur = nxt

        if cur == dst:
            break

    if cur != dst:
        raise RuntimeError(
            f"Torus route did not reach destination: src={src}, dst={dst}, cur={cur}, path={path}"
        )

    return path


def map_torus_traffic(n):
    """
    Full-torus traffic mapping.

    Returned arrays:
        h1_load[y,x]  = local horizontal link (x,y) <-> (x+1,y)
        v1_load[y,x]  = local vertical link   (x,y) <-> (x,y+1)
        hw_load[y]    = horizontal wrap link  (0,y) <-> (n-1,y)
        vw_load[x]    = vertical wrap link    (x,0) <-> (x,n-1)

    All counts are physical-link counts:
        count(u<->v) = count(u->v) + count(v->u)
    """
    h1_load = np.zeros((n, n - 1), dtype=int)
    v1_load = np.zeros((n - 1, n), dtype=int)
    hw_load = np.zeros(n, dtype=int)
    vw_load = np.zeros(n, dtype=int)

    nodes = all_nodes(n)
    total_flows = 0
    total_hops = 0

    for src in nodes:
        for dst in nodes:
            if src == dst:
                continue

            path = torus_route_path(src, dst, n)
            total_flows += 1
            total_hops += len(path)

            for u, v, _direction in path:
                ux, uy = u
                vx, vy = v

                if uy == vy:
                    # Horizontal local or wrap link
                    if abs(ux - vx) == 1:
                        h1_load[uy, min(ux, vx)] += 1
                    elif abs(ux - vx) == n - 1:
                        hw_load[uy] += 1
                    else:
                        raise RuntimeError(f"Illegal horizontal torus edge: {u}->{v}")
                elif ux == vx:
                    # Vertical local or wrap link
                    if abs(uy - vy) == 1:
                        v1_load[min(uy, vy), ux] += 1
                    elif abs(uy - vy) == n - 1:
                        vw_load[ux] += 1
                    else:
                        raise RuntimeError(f"Illegal vertical torus edge: {u}->{v}")
                else:
                    raise RuntimeError(f"Illegal diagonal torus edge: {u}->{v}")

    return h1_load, v1_load, hw_load, vw_load, total_flows, total_hops


# ============================================================
# Global scale helpers
# ============================================================

def max_load_for_topology_result(topology, result):
    if topology in ["mesh", "y_ruche_2hop", "full_ruche_2hop"]:
        arrays = result
        return max(int(a.max()) if a.size else 0 for a in arrays)
    if topology == "full_torus":
        h1, v1, hw, vw, _flows, _hops = result
        arrays = [h1, v1, hw, vw]
        return max(int(a.max()) if a.size else 0 for a in arrays)
    raise ValueError(f"Unknown topology for max-load calculation: {topology}")


def make_norm():
    cmap = colormaps["viridis_r"]  # low = green, high = red
    norm = colors.Normalize(vmin=GLOBAL_VMIN, vmax=GLOBAL_VMAX)
    return cmap, norm


# ============================================================
# Plot helpers
# ============================================================

def label_box(ax, x, y, text, color="black", edgecolor="none", fontsize=10):
    ax.text(
        x, y, str(text),
        ha="center",
        va="center",
        fontsize=fontsize,
        fontweight="bold",
        color=color,
        bbox=dict(
            boxstyle="round,pad=0.16",
            fc="white",
            ec=edgecolor,
            lw=0.8 if edgecolor != "none" else 0.0,
            alpha=0.96,
        ),
        zorder=20,
    )


def draw_nodes(ax, n):
    for x in range(n):
        for y in range(n):
            ax.scatter(x, -y, s=125, color="black", zorder=15)


def setup_axes(ax, n):
    ax.set_xlabel("x coordinate")
    ax.set_ylabel("y coordinate")
    ax.set_xticks(range(n))
    ax.set_yticks([-y for y in range(n)])
    ax.set_yticklabels([str(y) for y in range(n)])
    ax.set_xlim(-0.65, n - 1 + 0.65)
    ax.set_ylim(-(n - 1) - 0.55, 0.55)
    ax.set_aspect("equal")
    ax.grid(False)


def add_colorbar(fig, ax, cmap, norm):
    sm = plt.cm.ScalarMappable(norm=norm, cmap=cmap)
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label("Link usage count")

    if GLOBAL_VMAX == 32:
        ticks = [0, 8, 12, 16, 24, 32]
    else:
        ticks = sorted(set([
            0,
            int(round(GLOBAL_VMAX * 0.25)),
            int(round(GLOBAL_VMAX * 0.50)),
            int(round(GLOBAL_VMAX * 0.75)),
            int(round(GLOBAL_VMAX)),
        ]))
    cbar.set_ticks(ticks)


def draw_local_links(ax, n, h1_load, v1_load, cmap, norm):
    # H1 local horizontal links
    for y in range(n):
        for x in range(n - 1):
            val = int(h1_load[y, x])
            ax.plot(
                [x, x + 1],
                [-y, -y],
                color=cmap(norm(val)),
                linewidth=4.5,
                solid_capstyle="round",
                zorder=2,
            )
            label_box(ax, x + 0.5, -y, val)

    # V1 local vertical links
    for y in range(n - 1):
        for x in range(n):
            val = int(v1_load[y, x])
            ax.plot(
                [x, x],
                [-y, -(y + 1)],
                color=cmap(norm(val)),
                linewidth=3.4,
                solid_capstyle="round",
                zorder=2,
            )
            label_box(ax, x, -(y + 0.5) + 0.06, val)


# ============================================================
# Plot: mesh
# ============================================================

def plot_mesh(n, h1_load, v1_load, outdir):
    cmap, norm = make_norm()
    fig, ax = plt.subplots(figsize=(7.6, 7.6), dpi=180)

    draw_local_links(ax, n, h1_load, v1_load, cmap, norm)
    draw_nodes(ax, n)
    add_colorbar(fig, ax, cmap, norm)

    ax.set_title(
        f"{n}x{n} Mesh XY Routing: NoC-Channel Traffic Hot Map\n"
        f"Global color scale: {GLOBAL_VMIN}-{GLOBAL_VMAX}"
    )
    setup_axes(ax, n)

    png = outdir / f"{n}x{n}_mesh_xy_hotmap_global_scale.png"
    svg = outdir / f"{n}x{n}_mesh_xy_hotmap_global_scale.svg"
    fig.savefig(png, bbox_inches="tight", facecolor="white")
    fig.savefig(svg, bbox_inches="tight", facecolor="white")
    plt.close(fig)

    return [png, svg]


# ============================================================
# Plot: 2-hop Y half-ruche
# ============================================================

def plot_y_ruche_2hop(n, h1_load, v1_load, v2_load, outdir):
    cmap, norm = make_norm()
    fig, ax = plt.subplots(figsize=(8.2, 8.2), dpi=180)

    dash_style = (0, (4, 2.3))
    express_offset = 0.1
    end_gap = 0.06

    draw_local_links(ax, n, h1_load, v1_load, cmap, norm)

    # V2 express links
    for x in range(n):
        for y0 in range(n - 2):
            val = int(v2_load[y0, x])

            if y0 % 2 == 0:
                x_expr = x - express_offset
                text_x = x_expr - 0.075
            else:
                x_expr = x + express_offset
                text_x = x_expr + 0.075

            y_start = -y0 - end_gap
            y_end = -(y0 + 2) + end_gap
            y_mid = -(y0 + 1)

            ax.plot(
                [x_expr, x_expr],
                [y_start, y_end],
                color=cmap(norm(val)),
                linewidth=2.8,
                linestyle=dash_style,
                zorder=0,
            )

            label_box(
                ax,
                text_x,
                y_mid + 0.34,
                val,
                color="navy",
                edgecolor="navy",
                fontsize=10,
            )

    draw_nodes(ax, n)
    add_colorbar(fig, ax, cmap, norm)

    legend_elements = [
        Line2D([0], [0], color="black", lw=3.2, label="Solid = local 1-hop mesh link"),
        Line2D([0], [0], color="black", lw=2.8, linestyle=dash_style, label="Dashed = 2-hop Y express link"),
        Line2D(
            [0], [0],
            color="navy",
            lw=0,
            marker="s",
            markersize=8,
            markerfacecolor="white",
            markeredgecolor="navy",
            label="Blue number = express-link mapped count",
        ),
    ]

    ax.legend(
        handles=legend_elements,
        loc="upper center",
        bbox_to_anchor=(0.5, -0.08),
        frameon=False,
        ncol=1,
        fontsize=9,
    )

    ax.set_title(
        f"{n}x{n} 2-Hop Y-Direction Half-Ruche: NoC-Channel Traffic Hot Map\n"
        f"Global color scale: {GLOBAL_VMIN}-{GLOBAL_VMAX}"
    )
    setup_axes(ax, n)

    png = outdir / f"{n}x{n}_y_direction_2hop_half_ruche_hotmap_global_scale.png"
    svg = outdir / f"{n}x{n}_y_direction_2hop_half_ruche_hotmap_global_scale.svg"
    fig.savefig(png, bbox_inches="tight", facecolor="white")
    fig.savefig(svg, bbox_inches="tight", facecolor="white")
    plt.close(fig)

    return [png, svg]


# ============================================================
# Plot: 2-hop full-ruche
# ============================================================

def plot_full_ruche_2hop(n, h1_load, h2_load, v1_load, v2_load, outdir):
    cmap, norm = make_norm()
    fig, ax = plt.subplots(figsize=(9.0, 9.0), dpi=180)

    dash_style = (0, (4, 2.3))
    express_offset = 0.10
    end_gap = 0.06

    draw_local_links(ax, n, h1_load, v1_load, cmap, norm)

    # H2 express links:
    # H2[y,x] = (x,y) <-> (x+2,y)
    for y in range(n):
        for x0 in range(n - 2):
            val = int(h2_load[y, x0])

            if x0 % 2 == 0:
                y_expr = -y + express_offset + 0.02
                text_y = y_expr + 0.01
            else:
                y_expr = -y - express_offset
                text_y = y_expr - 0.01

            x_start = x0 + end_gap
            x_end = x0 + 2 - end_gap
            x_mid = x0 + 1

            ax.plot(
                [x_start, x_end],
                [y_expr, y_expr],
                color=cmap(norm(val)),
                linewidth=2.8,
                linestyle=dash_style,
                zorder=0,
            )

            label_box(
                ax,
                x_mid - 0.28,
                text_y,
                val,
                color="navy",
                edgecolor="navy",
                fontsize=10,
            )

    # V2 express links:
    # V2[y,x] = (x,y) <-> (x,y+2)
    for x in range(n):
        for y0 in range(n - 2):
            val = int(v2_load[y0, x])

            if y0 % 2 == 0:
                x_expr = x - express_offset
                text_x = x_expr - 0.02
            else:
                x_expr = x + express_offset
                text_x = x_expr + 0.02

            y_start = -y0 - end_gap
            y_end = -(y0 + 2) + end_gap
            y_mid = -(y0 + 1)

            ax.plot(
                [x_expr, x_expr],
                [y_start, y_end],
                color=cmap(norm(val)),
                linewidth=2.8,
                linestyle=dash_style,
                zorder=0,
            )

            label_box(
                ax,
                text_x,
                y_mid + 0.34,
                val,
                color="navy",
                edgecolor="navy",
                fontsize=10,
            )

    draw_nodes(ax, n)
    add_colorbar(fig, ax, cmap, norm)

    legend_elements = [
        Line2D([0], [0], color="black", lw=3.2, label="Solid = local 1-hop mesh link"),
        Line2D([0], [0], color="black", lw=2.8, linestyle=dash_style, label="Dashed = 2-hop express link"),
        Line2D(
            [0], [0],
            color="navy",
            lw=0,
            marker="s",
            markersize=8,
            markerfacecolor="white",
            markeredgecolor="navy",
            label="Blue number = express-link mapped count",
        ),
    ]

    ax.legend(
        handles=legend_elements,
        loc="upper center",
        bbox_to_anchor=(0.5, -0.08),
        frameon=False,
        ncol=1,
        fontsize=9,
    )

    ax.set_title(
        f"{n}x{n} 2-Hop Full-Ruche: NoC-Channel Traffic Hot Map\n"
        f"Global color scale: {GLOBAL_VMIN}-{GLOBAL_VMAX}"
    )
    setup_axes(ax, n)

    png = outdir / f"{n}x{n}_2hop_full_ruche_hotmap_global_scale.png"
    svg = outdir / f"{n}x{n}_2hop_full_ruche_hotmap_global_scale.svg"
    fig.savefig(png, bbox_inches="tight", facecolor="white")
    fig.savefig(svg, bbox_inches="tight", facecolor="white")
    plt.close(fig)

    return [png, svg]


# ============================================================
# Plot: full torus
# ============================================================

def plot_full_torus(n, h1_load, v1_load, hw_load, vw_load, total_flows, total_hops, outdir):
    """
    Plot full-torus with the same visual grammar:
      - solid = local 1-hop mesh link
      - dashed = torus wrap-around link
      - blue boxed number = wrap-link mapped count
    """
    cmap, norm = make_norm()
    fig, ax = plt.subplots(figsize=(9.0, 9.0), dpi=180)

    dash_style = (0, (4, 2.3))
    end_gap = 0.08

    # Local links reuse the mesh visual base.
    draw_local_links(ax, n, h1_load, v1_load, cmap, norm)

    # Horizontal wrap links:
    # HW[y] = (0,y) <-> (n-1,y)
    for y in range(n):
        val = int(hw_load[y])

        # Display y coordinate is -y.
        # Rows in the upper half are drawn above; lower half rows are drawn below.
        if y < n / 2:
            y_out = -y + 0.15
        else:
            y_out = -y - 0.15

        ax.plot(
            [0, 0],
            [-y, y_out],
            color=cmap(norm(val)),
            linewidth=2.8,
            linestyle=dash_style,
            zorder=0,
        )
        ax.plot(
            [0 + end_gap, n - 1 - end_gap],
            [y_out, y_out],
            color=cmap(norm(val)),
            linewidth=2.8,
            linestyle=dash_style,
            zorder=0,
        )
        ax.plot(
            [n - 1, n - 1],
            [-y, y_out],
            color=cmap(norm(val)),
            linewidth=2.8,
            linestyle=dash_style,
            zorder=0,
        )

        label_box(
            ax,
            (n - 1) / 2,
            y_out,
            val,
            color="navy",
            edgecolor="navy",
            fontsize=10,
        )

    # Vertical wrap links:
    # VW[x] = (x,0) <-> (x,n-1)
    for x in range(n):
        val = int(vw_load[x])

        if x < n / 2:
            x_out = x - 0.15
        else:
            x_out = x + 0.15

        ax.plot(
            [x, x_out],
            [0, 0],
            color=cmap(norm(val)),
            linewidth=2.8,
            linestyle=dash_style,
            zorder=0,
        )
        ax.plot(
            [x_out, x_out],
            [-0 - end_gap, -(n - 1) + end_gap],
            color=cmap(norm(val)),
            linewidth=2.8,
            linestyle=dash_style,
            zorder=0,
        )
        ax.plot(
            [x, x_out],
            [-(n - 1), -(n - 1)],
            color=cmap(norm(val)),
            linewidth=2.8,
            linestyle=dash_style,
            zorder=0,
        )

        label_box(
            ax,
            x_out,
            -(n - 1) / 2,
            val,
            color="navy",
            edgecolor="navy",
            fontsize=10,
        )

    draw_nodes(ax, n)
    add_colorbar(fig, ax, cmap, norm)

    legend_elements = [
        Line2D([0], [0], color="black", lw=3.2, label="Solid = local 1-hop mesh link"),
        Line2D([0], [0], color="black", lw=2.8, linestyle=dash_style, label="Dashed = torus wrap-around link"),
        Line2D(
            [0], [0],
            color="navy",
            lw=0,
            marker="s",
            markersize=8,
            markerfacecolor="white",
            markeredgecolor="navy",
            label="Blue number = wrap-link mapped count",
        ),
    ]

    ax.legend(
        handles=legend_elements,
        loc="upper center",
        bbox_to_anchor=(0.5, -0.08),
        frameon=False,
        ncol=1,
        fontsize=9,
    )

    ax.set_title(
        f"{n}x{n} Full-Torus: NoC-Channel Traffic Hot Map\n"
        f"Global color scale: {GLOBAL_VMIN}-{GLOBAL_VMAX}, "
        f"avg. hop = {total_hops / total_flows:.3f}"
    )
    setup_axes(ax, n)

    png = outdir / f"{n}x{n}_full_torus_hotmap_global_scale.png"
    svg = outdir / f"{n}x{n}_full_torus_hotmap_global_scale.svg"
    fig.savefig(png, bbox_inches="tight", facecolor="white")
    fig.savefig(svg, bbox_inches="tight", facecolor="white")
    plt.close(fig)

    return [png, svg]


# ============================================================
# Main
# ============================================================

def selected_topologies_from_arg(topology_arg):
    if topology_arg == "mesh":
        return ["mesh"]
    if topology_arg == "y_ruche_2hop":
        return ["y_ruche_2hop"]
    if topology_arg == "full_ruche_2hop":
        return ["full_ruche_2hop"]
    if topology_arg == "full_torus":
        return ["full_torus"]
    if topology_arg == "both":
        return ["mesh", "y_ruche_2hop"]
    if topology_arg == "all_no_torus":
        return ["mesh", "y_ruche_2hop", "full_ruche_2hop"]
    if topology_arg == "all":
        return ["mesh", "y_ruche_2hop", "full_ruche_2hop", "full_torus"]
    raise ValueError(f"Unknown topology argument: {topology_arg}")


def main():
    global GLOBAL_VMAX

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--N",
        type=int,
        default=4,
        help="Mesh size. Default: 4.",
    )

    parser.add_argument(
        "--topology",
        choices=[
            "mesh",
            "y_ruche_2hop",
            "full_ruche_2hop",
            "full_torus",
            "both",
            "all_no_torus",
            "all",
        ],
        default="all",
        help=(
            "Topology to plot. "
            "'both' means mesh + y_ruche_2hop. "
            "'all_no_torus' means mesh + y_ruche_2hop + full_ruche_2hop. "
            "'all' means mesh + y_ruche_2hop + full_ruche_2hop + full_torus."
        ),
    )

    parser.add_argument(
        "--outdir",
        type=str,
        default=".",
        help="Output directory.",
    )

    parser.add_argument(
        "--global-vmax",
        type=str,
        default="32",
        help=(
            "Global color-scale maximum shared by all selected topologies. "
            "Use a number, e.g. 32, or 'auto' to use the maximum link load "
            "over all selected topologies. Default: 32."
        ),
    )

    args = parser.parse_args()

    n = args.N
    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    if n < 2:
        raise ValueError("N must be >= 2.")
    if n < 3 and args.topology in ["y_ruche_2hop", "full_ruche_2hop", "all", "both", "all_no_torus"]:
        raise ValueError("2-hop ruche requires N >= 3.")

    selected = selected_topologies_from_arg(args.topology)

    # First map traffic for every selected topology.
    # This is what allows a true shared global scale.
    results = {}

    for topo in selected:
        if topo in ["mesh", "y_ruche_2hop", "full_ruche_2hop"]:
            results[topo] = map_mesh_ruche_traffic(n, topo)
        elif topo == "full_torus":
            results[topo] = map_torus_traffic(n)
        else:
            raise ValueError(f"Unknown selected topology: {topo}")

    # Decide global color scale after all selected topologies are known.
    if args.global_vmax.lower() == "auto":
        GLOBAL_VMAX = max(max_load_for_topology_result(topo, res) for topo, res in results.items())
    else:
        GLOBAL_VMAX = int(args.global_vmax)

    if GLOBAL_VMAX <= GLOBAL_VMIN:
        raise ValueError("GLOBAL_VMAX must be larger than GLOBAL_VMIN.")

    outputs = []

    if "mesh" in selected:
        h1, h2, v1, v2 = results["mesh"]

        print("\n=== Mesh XY routing ===")
        print("H1 local horizontal mesh links H1[y,x]:")
        print(h1)
        print("V1 local vertical mesh links V1[y,x]:")
        print(v1)

        outputs += plot_mesh(n, h1, v1, outdir)

    if "y_ruche_2hop" in selected:
        h1, h2, v1, v2 = results["y_ruche_2hop"]

        print("\n=== 2-hop Y-direction half-ruche ===")
        print("H1 local horizontal mesh links H1[y,x]:")
        print(h1)
        print("V1 local vertical mesh links V1[y,x]:")
        print(v1)
        print("V2 vertical 2-hop express links V2[y,x]:")
        print("V2[y,x] = express link (x,y) <-> (x,y+2)")
        print(v2)

        outputs += plot_y_ruche_2hop(n, h1, v1, v2, outdir)

    if "full_ruche_2hop" in selected:
        h1, h2, v1, v2 = results["full_ruche_2hop"]

        print("\n=== 2-hop full-ruche ===")
        print("H1 local horizontal mesh links H1[y,x]:")
        print(h1)
        print("H2 horizontal 2-hop express links H2[y,x]:")
        print("H2[y,x] = express link (x,y) <-> (x+2,y)")
        print(h2)
        print("V1 local vertical mesh links V1[y,x]:")
        print(v1)
        print("V2 vertical 2-hop express links V2[y,x]:")
        print("V2[y,x] = express link (x,y) <-> (x,y+2)")
        print(v2)

        outputs += plot_full_ruche_2hop(n, h1, h2, v1, v2, outdir)

    if "full_torus" in selected:
        h1, v1, hw, vw, total_flows, total_hops = results["full_torus"]

        print("\n=== Full torus, routing-table-style X-first shortest torus routing ===")
        print("H1 local horizontal mesh links H1[y,x]:")
        print(h1)
        print("V1 local vertical mesh links V1[y,x]:")
        print(v1)
        print("HW horizontal wrap links HW[y]:")
        print("HW[y] = wrap link (0,y) <-> (N-1,y)")
        print(hw)
        print("VW vertical wrap links VW[x]:")
        print("VW[x] = wrap link (x,0) <-> (x,N-1)")
        print(vw)
        print(f"Active ordered flows: {total_flows}")
        print(f"Total routed hops: {total_hops}")
        print(f"Average hop count: {total_hops / total_flows:.6f}")

        outputs += plot_full_torus(n, h1, v1, hw, vw, total_flows, total_hops, outdir)

    print(f"\nGlobal color scale shared by selected topologies: {GLOBAL_VMIN}-{GLOBAL_VMAX}")

    print("\nGenerated files:")
    for path in outputs:
        print(f"  {path}")


if __name__ == "__main__":
    main()
