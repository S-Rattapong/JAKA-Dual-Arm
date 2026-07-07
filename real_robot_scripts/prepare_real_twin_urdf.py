#!/usr/bin/env python3
import argparse
import xml.etree.ElementTree as ET
from pathlib import Path


def parse_xyz(text):
    vals = [float(x) for x in text.split()]
    while len(vals) < 3:
        vals.append(0.0)
    return vals[:3]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--prefix", default="real_")
    parser.add_argument("--y-shift", type=float, default=1.2)
    args = parser.parse_args()

    in_path = Path(args.input).expanduser()
    out_path = Path(args.output).expanduser()

    tree = ET.parse(in_path)
    root = tree.getroot()
    root.set("name", "dual_jaka_a12_real_twin")

    # Rename all links except world.
    for link in root.findall("link"):
        old = link.attrib.get("name", "")
        if old != "world":
            link.attrib["name"] = args.prefix + old

    # Rename joints and parent/child links.
    for joint in root.findall("joint"):
        old_joint = joint.attrib.get("name", "")
        joint.attrib["name"] = args.prefix + old_joint

        parent_is_world = False

        parent = joint.find("parent")
        if parent is not None:
            p = parent.attrib.get("link", "")
            if p == "world":
                parent_is_world = True
            else:
                parent.attrib["link"] = args.prefix + p

        child = joint.find("child")
        if child is not None:
            c = child.attrib.get("link", "")
            if c != "world":
                child.attrib["link"] = args.prefix + c

        # Shift only joints directly attached to world, so real twin appears beside sim model.
        if parent_is_world:
            origin = joint.find("origin")
            if origin is None:
                origin = ET.SubElement(joint, "origin")
                origin.attrib["xyz"] = f"0 {args.y_shift} 0"
                origin.attrib["rpy"] = "0 0 0"
            else:
                xyz = parse_xyz(origin.attrib.get("xyz", "0 0 0"))
                xyz[1] += args.y_shift
                origin.attrib["xyz"] = f"{xyz[0]} {xyz[1]} {xyz[2]}"
                if "rpy" not in origin.attrib:
                    origin.attrib["rpy"] = "0 0 0"

    out_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(out_path, encoding="utf-8", xml_declaration=True)
    print(f"Saved real twin URDF to: {out_path}")


if __name__ == "__main__":
    main()
