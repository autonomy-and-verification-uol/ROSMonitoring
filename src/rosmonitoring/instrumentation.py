from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET


def add_ros1_remaps(launch_path: str | Path, node_name: str, remaps: dict[str, str]) -> bool:
    """Add missing remap tags to a ROS1 XML launch file.

    Returns True when the file was changed. The function is intentionally small
    and deterministic so it can be used in tests and reviewed before applying
    invasive instrumentation to application launch files.
    """
    path = Path(launch_path)
    tree = ET.parse(path)
    root = tree.getroot()
    changed = False
    for node in root.iter("node"):
        if node.attrib.get("name") != node_name:
            continue
        existing = {(child.attrib.get("from"), child.attrib.get("to")) for child in node.findall("remap")}
        for src, dst in remaps.items():
            if (src, dst) in existing:
                continue
            ET.SubElement(node, "remap", {"from": src, "to": dst})
            changed = True
    if changed:
        tree.write(path, encoding="unicode")
    return changed
