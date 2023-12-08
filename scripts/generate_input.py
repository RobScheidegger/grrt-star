"""
This file generates input YAML files for the solver, based on
"""

import argparse
import os
import yaml
import math
import random

argparser = argparse.ArgumentParser(
    description="Generate input YAML files for the solver."
)

argparser.add_argument(
    "-o",
    "--output",
    type=str,
    help="Output file name.",
    required=True,
)

argparser.add_argument(
    "--random",
    help="Whether to generate a random graph.",
    action="store_true",
)

argparser.add_argument(
    "-t",
    "--type",
    type=str,
    help="Type of the graph.",
    required=True,
)

argparser.add_argument(
    "-n",
    "--n",
    type=int,
    help="Number of nodes in the graph.",
    required=True,
)

argparser.add_argument(
    "-r", "--radius", type=int, help="Radius of the graph.", required=False, default=1
)

argparser.add_argument(
    "-s",
    "--solution",
    help="Whether to generate a solution.",
    action="store_true",
)

args = argparser.parse_args()

contents = None
solution = None

if args.type == "circle_rotate":
    circle_positions = [
        (
            args.radius * math.cos(2 * math.pi * i / args.n),
            args.radius * math.sin(2 * math.pi * i / args.n),
            0,
        )
        for i in range(args.n)
    ] + [
        (
            args.radius * math.cos(2 * math.pi * i / args.n),
            args.radius * math.sin(2 * math.pi * i / args.n),
            5,
        )
        for i in range(args.n)
    ]

    start_positions = {
        "robot_{}".format(i): "state_{}".format(i) for i in range(args.n)
    }

    end_positions = {
        "robot_{}".format(i): "state_{}".format((i + 1) % args.n) for i in range(args.n)
    }

    contents = {
        "voxels": {"type": "PCL"},
        "robots": [
            {"name": "robot_{}".format(i), "type": "drone", "roadmap": "roadmap"}
            for i in range(args.n)
        ],
        "roadmaps": [
            {
                "name": "roadmap",
                "type": "drone",
                "states": [
                    {
                        "name": "state_{}".format(i),
                        "to": [
                            "state_{}".format((i + 1) % args.n),
                            "state_{}".format(i),
                            "state_{}".format((i - 1) % args.n),
                            "state_{}".format((i + args.n) % (2 * args.n)),
                        ],
                        "x": circle_positions[i][0],
                        "y": circle_positions[i][1],
                        "z": circle_positions[i][2],
                    }
                    for i in range(2 * args.n)
                ],
            }
        ],
        "problems": [
            {"name": "problem", "start": start_positions, "goal": end_positions}
        ],
    }
    if args.solution:
        if args.random:
            # Generate a random permutation of the nodes

            order = list(range(2 * args.n))
            solution = "0\n0\n"
            for _ in range(7):
                solution += ",".join([str(i) for i in order])
                solution += "\n"
                random.shuffle(order)
        else:
            solution = "0\n0\n"
            solution += ",".join([str(i) for i in range(args.n)])
            solution += "\n"
            solution += ",".join([str((i + 1) % args.n) for i in range(args.n)])

# Write contents to a YAML file

with open(args.output, "w") as f:
    yaml.dump(contents, f)

if args.solution:
    with open(args.output + ".sol", "w") as f:
        f.write(solution)
