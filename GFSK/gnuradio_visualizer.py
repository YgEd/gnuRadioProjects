#!/usr/bin/env python3

import inspect
from gnuradio import gr


def visualize_flowgraph(tb, outfile="flowgraph.dot"):
    """
    Generate a Graphviz .dot file from a GNU Radio top_block
    """

    # Map block object -> variable name (best effort)
    block_names = {}

    # Walk the caller's frame to find variable names
    frame = inspect.currentframe().f_back
    for name, obj in frame.f_locals.items():
        if isinstance(obj, gr.basic_block):
            block_names[obj] = name

    lines = []
    lines.append("digraph gnuradio {")
    lines.append('  rankdir=LR;')
    lines.append('  node [shape=box, style=rounded];')

    # Emit nodes
    for block in tb.blocks():
        label = block_names.get(block, block.name())
        lines.append(f'  "{id(block)}" [label="{label}"];')

    # Emit edges
    for src, src_port, dst, dst_port in tb.connections():
        src_label = f"{block_names.get(src, src.name())}:{src_port}"
        dst_label = f"{block_names.get(dst, dst.name())}:{dst_port}"

        lines.append(
            f'  "{id(src)}" -> "{id(dst)}" '
            f'[label="{src_port} → {dst_port}"];'
        )

    lines.append("}")

    with open(outfile, "w") as f:
        f.write("\n".join(lines))

    print(f"[+] Wrote {outfile}")
    print("    Render with: dot -Tpng flowgraph.dot -o flowgraph.png")
