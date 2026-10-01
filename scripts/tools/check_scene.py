#!/usr/bin/env python3
"""Validate the SDF scene Gazebo will consume, not merely its XML numbers.

Each collision and visual must contain exactly one geometry with one box. Its
pose belongs to the collision/visual, never inside a box. This structural
check catches the former bins: five boxes under one geometry looked correct to
an XML AABB checker but became one floor slab in libsdformat/Gazebo.

Usage: python3 scripts/tools/check_scene.py [--world PATH]
"""

import argparse
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

TABLE_TOP_Z = 0.35
BIN_OUTER = 0.13
BASE_POSE = (0.0, 0.0, 0.347, 0.0)  # gazebo.launch.py spawn arguments
MAX_HORIZONTAL_REACH = 0.626  # only a coarse bound; use MoveIt for real IK
ROOT = Path(__file__).resolve().parents[2]
DEFAULT_WORLD = ROOT / "src/description/classify_description/worlds/classify.world"
ROBOT_URDF = ROOT / "src/description/classify_description/urdf/piper_sim.urdf"


def read_pose(element):
    raw = element.findtext("pose", "0 0 0 0 0 0")
    values = [float(v) for v in raw.split()]
    if len(values) != 6:
        raise ValueError(f"{element.tag} pose needs six numbers: {raw!r}")
    x, y, z, roll, pitch, yaw = values
    if abs(roll) > 1e-9 or abs(pitch) > 1e-9:
        raise ValueError(f"{element.tag} roll/pitch pose unsupported by this checker: {raw}")
    return x, y, z, yaw


def combine(parent, child):
    x, y, z, yaw = parent
    dx, dy, dz, dyaw = child
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * dx - s * dy, y + s * dx + c * dy, z + dz, yaw + dyaw


def box_aabb(pose, dimensions):
    x, y, z, yaw = pose
    sx, sy, sz = dimensions
    c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
    half_x = (c * sx + s * sy) / 2
    half_y = (s * sx + c * sy) / 2
    return x - half_x, x + half_x, y - half_y, y + half_y, z - sz / 2, z + sz / 2


def union(boxes):
    return (min(b[0] for b in boxes), max(b[1] for b in boxes),
            min(b[2] for b in boxes), max(b[3] for b in boxes),
            min(b[4] for b in boxes), max(b[5] for b in boxes))


def overlaps_xy(a, b, tolerance=1e-6):
    return min(a[1], b[1]) - max(a[0], b[0]) > tolerance and \
        min(a[3], b[3]) - max(a[2], b[2]) > tolerance


class Scene:
    def __init__(self, path):
        self.errors = []
        self.root = ET.parse(path).getroot()
        self.models = {m.get("name"): m for m in self.root.findall("world/model")}

    def require(self, condition, message):
        if not condition:
            self.errors.append(message)

    def model(self, name):
        model = self.models.get(name)
        self.require(model is not None, f"missing model {name}")
        return model

    def shapes(self, model_name, link_name=None, kind="collision"):
        """Return actual box AABBs, keyed by collision/visual name."""
        model = self.model(model_name)
        if model is None:
            return {}
        links = model.findall("link")
        if link_name is not None:
            links = [l for l in links if l.get("name") == link_name]
            self.require(bool(links), f"{model_name}: missing link {link_name}")
        result = {}
        for link in links:
            lp = combine(read_pose(model), read_pose(link))
            for item in link.findall(kind):
                title = f"{model_name}/{link.get('name')}/{kind}/{item.get('name')}"
                key = f"{link.get('name')}::{item.get('name')}"
                self.require(item.get("name") is not None, f"{title}: missing name")
                self.require(key not in result, f"{title}: duplicate name")
                geometry = item.findall("geometry")
                if len(geometry) != 1:
                    self.errors.append(f"{title}: expected exactly one geometry")
                    continue
                shapes = list(geometry[0])
                if len(shapes) != 1 or shapes[0].tag != "box":
                    self.errors.append(f"{title}: geometry must contain exactly one box")
                    continue
                box = shapes[0]
                if box.find("pose") is not None:
                    self.errors.append(f"{title}: pose inside box is invalid; move it to {kind}/pose")
                    continue
                size = box.findtext("size")
                if size is None:
                    self.errors.append(f"{title}: missing box size")
                    continue
                try:
                    dims = [float(v) for v in size.split()]
                    if len(dims) != 3 or any(v <= 0 for v in dims):
                        raise ValueError
                except ValueError:
                    self.errors.append(f"{title}: invalid box size {size!r}")
                    continue
                result[key] = box_aabb(combine(lp, read_pose(item)), dims)
        return result

    def check(self):
        for name in ("ground_plane", "table", "red_block", "green_block",
                     "blue_block", "bin_red", "bin_blue"):
            self.model(name)
        if "table" not in self.models:
            return self.errors

        # The agreed work direction is world +X, matching Piper's HOME reach.
        # A model-level yaw is deliberate: it makes the table's Gazebo pose
        # visibly rotated instead of only rewriting individual link numbers.
        table_yaw = read_pose(self.models["table"])[3]
        self.require(abs(table_yaw + math.pi / 2) < 1e-5,
                     f"table model yaw={math.degrees(table_yaw):+.1f}°, expected -90°")

        table = self.shapes("table", "table_top_link")
        self.require(len(table) == 1, "table top needs exactly one collision box")
        if not table:
            return self.errors
        top = next(iter(table.values()))
        self.require(abs(top[5] - TABLE_TOP_Z) < 1e-6,
                     f"table top z={top[5]:.4f}, expected {TABLE_TOP_Z:.4f}")
        print(f"table: x [{top[0]:+.3f}, {top[1]:+.3f}], y [{top[2]:+.3f}, {top[3]:+.3f}], top z={top[5]:.3f}")

        legs = self.models["table"].findall("link")
        legs = [l for l in legs if l.get("name", "").startswith("leg_")]
        self.require(len(legs) == 4, f"table needs four legs; got {len(legs)}")
        for leg in legs:
            name = leg.get("name")
            shapes = self.shapes("table", name)
            self.require(len(shapes) == 1, f"{name} needs one collision box")
            if shapes:
                b = next(iter(shapes.values()))
                self.require(abs(b[4]) < 1e-6, f"{name} floats/sinks at z={b[4]:.4f}")
                self.require(abs(b[5] - top[4]) < 1e-6,
                             f"{name} does not meet underside of tabletop")
                self.require(top[0] <= b[0] and b[1] <= top[1] and
                             top[2] <= b[2] and b[3] <= top[3],
                             f"{name} extends beyond tabletop")

        block_boxes = {}
        for name in ("red_block", "green_block", "blue_block"):
            collision = self.shapes(name)
            visual = self.shapes(name, kind="visual")
            self.require(len(collision) == 1 and len(visual) == 1,
                         f"{name} needs one collision and one visual")
            if not collision:
                continue
            b = next(iter(collision.values()))
            block_boxes[name] = b
            self.require(abs(b[4] - top[5]) < 1e-6,
                         f"{name} bottom z={b[4]:.4f} vs table {top[5]:.4f}")
            self.require(top[0] <= b[0] and b[1] <= top[1] and
                         top[2] <= b[2] and b[3] <= top[3],
                         f"{name} extends beyond tabletop")
            self.check_reach(name, b)
            print(f"{name}: x [{b[0]:+.3f}, {b[1]:+.3f}], y [{b[2]:+.3f}, {b[3]:+.3f}], bottom z={b[4]:.3f}")

        bin_boxes = {}
        for name in ("bin_red", "bin_blue"):
            collision = self.shapes(name)
            visual = self.shapes(name, kind="visual")
            self.require(len(collision) == 5 and len(visual) == 5,
                         f"{name} needs five separate collision and visual boxes")
            for i in ("floor", "south", "north", "east", "west"):
                c = collision.get(f"link::collision_{i}")
                v = visual.get(f"link::visual_{i}")
                self.require(c is not None and v is not None,
                             f"{name} missing {i} collision/visual")
                if c is not None and v is not None:
                    self.require(all(abs(x - y) < 1e-6 for x, y in zip(c, v)),
                                 f"{name} {i} visual differs from collision")
            if not collision:
                continue
            b = union(collision.values())
            bin_boxes[name] = b
            self.require(abs(b[0] - b[1] + BIN_OUTER) < 1e-6 and
                         abs(b[2] - b[3] + BIN_OUTER) < 1e-6,
                         f"{name} outer footprint is not {BIN_OUTER} m square")
            floor = collision.get("link::collision_floor")
            if floor:
                self.require(abs(floor[4] - top[5]) < 1e-6,
                             f"{name} floor bottom z={floor[4]:.4f} vs table {top[5]:.4f}")
                for i in ("south", "north", "east", "west"):
                    wall = collision.get(f"link::collision_{i}")
                    if wall:
                        self.require(wall[4] <= floor[5] + 1e-6 and
                                     wall[4] >= floor[4] - 1e-6,
                                     f"{name} {i} wall is detached from floor")
            self.require(top[0] <= b[0] and b[1] <= top[1] and
                         top[2] <= b[2] and b[3] <= top[3],
                         f"{name} extends beyond tabletop")
            self.check_reach(name, b)
            print(f"{name}: x [{b[0]:+.3f}, {b[1]:+.3f}], y [{b[2]:+.3f}, {b[3]:+.3f}], bottom z={b[4]:.3f}, pieces={len(collision)}")

        all_boxes = {**block_boxes, **bin_boxes}
        names = list(all_boxes)
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                self.require(not overlaps_xy(all_boxes[a], all_boxes[b]),
                             f"{a} overlaps {b} in XY")

        base = self.base_box()
        if base:
            self.require(abs(base[4] - top[5]) < 1e-6,
                         f"robot base bottom z={base[4]:.4f} vs table {top[5]:.4f}")
            self.require(top[0] <= base[0] and base[1] <= top[1] and
                         top[2] <= base[2] and base[3] <= top[3],
                         "robot base extends beyond tabletop")
            for b in all_boxes:
                self.require(not overlaps_xy(base, all_boxes[b]),
                             f"robot base overlaps {b} in XY")
        return self.errors

    def check_reach(self, name, box):
        x, y = (box[0] + box[1]) / 2, (box[2] + box[3]) / 2
        r = math.hypot(x - BASE_POSE[0], y - BASE_POSE[1])
        self.require(x > BASE_POSE[0] + 0.10,
                     f"{name} center x={x:.3f} is not in the agreed +X work area")
        self.require(r <= MAX_HORIZONTAL_REACH,
                     f"{name} center r={r:.3f} exceeds coarse reach bound {MAX_HORIZONTAL_REACH}")

    def base_box(self):
        robot = ET.parse(ROBOT_URDF).getroot()
        base = robot.find("link[@name='base_link']/collision")
        if base is None:
            self.errors.append("robot URDF has no base_link collision")
            return None
        size = base.find("geometry/box")
        origin = base.find("origin")
        if size is None or origin is None:
            self.errors.append("robot base collision is not a box with an origin")
            return None
        dims = [float(x) for x in size.get("size").split()]
        xyz = [float(x) for x in origin.get("xyz").split()]
        return box_aabb(combine(BASE_POSE, (*xyz, 0.0)), dims)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", type=Path, default=DEFAULT_WORLD)
    args = parser.parse_args()
    try:
        scene = Scene(args.world)
        errors = scene.check()
    except (ET.ParseError, ValueError) as exc:
        print(f"INVALID SDF/XML: {exc}", file=sys.stderr)
        return 1
    if errors:
        for error in errors:
            print(f"FAIL: {error}", file=sys.stderr)
        print(f"scene check failed: {len(errors)} issue(s)", file=sys.stderr)
        return 1
    print("scene check passed: structural and geometry checks")
    return 0


if __name__ == "__main__":
    sys.exit(main())
