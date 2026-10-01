#!/usr/bin/env python3
"""Rotate every scene model about world Z as one rigid layout.

SDF's model pose transforms all of its child links, visuals, collisions and
sensors. Only the model pose should change: manually rotating child poses and
swapping box extents creates duplicate transforms and is difficult to audit.

Examples:
  python3 scripts/tools/rotate_scene.py --world PATH --angle -90 --dry-run
  python3 scripts/tools/rotate_scene.py --world PATH --angle -90

The initial scene faced +Y. -90 degrees rotates it clockwise toward +X, the
direction of the robot's HOME reach. Ground plane is rotation invariant.
"""

import argparse
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


def fmt(value):
    if abs(value) < 1e-10:
        return "0"
    return f"{value:.12g}"


def rotate(model, angle):
    pose = model.find("pose")
    values = [0.0] * 6 if pose is None else [float(x) for x in (pose.text or "").split()]
    if len(values) != 6:
        raise ValueError(f"{model.get('name')}: model pose needs six numbers")
    x, y, z, roll, pitch, yaw = values
    if abs(roll) > 1e-9 or abs(pitch) > 1e-9:
        raise ValueError(f"{model.get('name')}: model roll/pitch is not zero")
    c, s = math.cos(angle), math.sin(angle)
    nx, ny = c * x - s * y, s * x + c * y
    nyaw = math.atan2(math.sin(yaw + angle), math.cos(yaw + angle))
    if pose is None:
        pose = ET.Element("pose")
        # Place before child links, as in ordinary SDF authoring.
        static = model.find("static")
        model.insert(list(model).index(static) + 1 if static is not None else 0, pose)
    pose.text = f"{fmt(nx)} {fmt(ny)} {fmt(z)} 0 0 {fmt(nyaw)}"
    return (x, y, yaw), (nx, ny, nyaw)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", type=Path, required=True)
    parser.add_argument("--angle", type=float, required=True, help="degrees; -90 is clockwise")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    if args.angle % 90 != 0:
        parser.error("angle must be a multiple of 90 degrees for this box scene")
    xml_parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(args.world, parser=xml_parser)
    models = tree.getroot().findall("world/model")
    required = {"table", "red_block", "green_block", "blue_block", "bin_red", "bin_blue"}
    names = {m.get("name") for m in models}
    if not required <= names:
        raise ValueError(f"scene is missing models: {sorted(required - names)}")
    angle = math.radians(args.angle)
    for model in models:
        if model.get("name") == "ground_plane":
            continue
        before, after = rotate(model, angle)
        print(f"{model.get('name'):12s}: ({before[0]:+.3f}, {before[1]:+.3f}, yaw {math.degrees(before[2]):+.1f}°)"
              f" -> ({after[0]:+.3f}, {after[1]:+.3f}, yaw {math.degrees(after[2]):+.1f}°)")
    if args.dry_run:
        print("dry-run: no file written")
        return 0
    ET.indent(tree, space="  ")
    tree.write(args.world, encoding="utf-8", xml_declaration=True)
    print(f"wrote {args.world}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
