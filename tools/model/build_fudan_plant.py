#!/usr/bin/env python3
"""Build the Fudan robot into the repository's existing MuJoCo plant shell."""

from __future__ import annotations

import copy
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

from build_fudan_collision_proxies import (
    PROXY_SPECS,
    build_collision_proxy_assets,
    proxy_file_name,
    proxy_mesh_name,
)


ROOT = Path(__file__).resolve().parents[2]
COD_MODEL = ROOT / "models" / "MJCF" / "COD-2026RoboMaster-Balance.xml"
FUDAN_MODEL = (
    ROOT / "references" / "fudan_rl_wheel_leg" / "mujoco" /
    "assert_now" / "infantry_binglian_yuntai" / "infantry_V2" /
    "meshes" / "mjmodel.xml"
)
OUTPUT = ROOT / "models" / "MJCF" / "Fudan-2026RoboMaster-Balance.xml"
ASSET_DIR = ROOT / "models" / "MJCF" / "fudan"
SOURCE_ASSET_DIR = FUDAN_MODEL.parent


BODY_NAMES = {
    "base_Link_del": "base_link",
    "l20_Link": "Right_front_link",
    "l21_Link": "Right_front_child1_link",
    "l22_Link": "Right_front_child2_link",
    "l23_Link": "Right_front_child3_link",
    "lf0_Link": "Right_rear_link",
    "lf1_Link": "Right_rear_child1_link",
    "l_wheel_Link": "Right_Wheel_link",
    "r20_Link": "Left_front_link",
    "r21_Link": "Left_front_child1_link",
    "r22_Link": "Left_front_child2_link",
    "r23_Link": "Left_front_child3_link",
    "rf0_Link": "Left_rear_link",
    "rf1_Link": "Left_rear_child1_link",
    "r_wheel_Link": "Left_Wheel_link",
}

JOINT_NAMES = {
    "l20_Joint": "Right_front_joint",
    "l21_Joint": "Right_front_child1_joint",
    "l21_Link": "Right_front_child1_joint",
    "l22_Joint": "Right_front_child2_joint",
    "l23_Joint": "Right_front_joint3_joint",
    "lf0_Joint": "Right_rear_joint",
    "lf1_Joint": "Right_rear_child1_joint",
    "l_wheel_Joint": "Right_Wheel_joint",
    "r20_Joint": "Left_front_joint",
    "r21_Joint": "Left_front_child1_joint",
    "r21_Link": "Left_front_child1_joint",
    "r22_Joint": "Left_front_child2_joint",
    "r23_Joint": "Left_front_child3_joint",
    "rf0_Joint": "Left_rear_joint",
    "rf1_Joint": "Left_rear_child1_joint",
    "r_wheel_Joint": "Left_Wheel_joint",
}

SITE_NAMES = {
    "Left_front_site1": "Right_rear_site1",
    "Left_front_site2": "Right_rear_site2",
    "Left_rear_site1": "Right_front_site1",
    "Left_rear_site2": "Right_front_site2",
    "Right_front_site1": "Left_rear_site1",
    "Right_front_site2": "Left_rear_site2",
    "Right_rear_site1": "Left_front_site1",
    "Right_rear_site2": "Left_front_site2",
}

MESH_FILES = {
    "base_link": "fudan/fudan_base_link_del.STL",
    "Left_front_link": "fudan/r20_Link.STL",
    "Left_front_child1_link": "fudan/r21_Link.STL",
    "Left_front_child2_link": "fudan/r22_Link.STL",
    "Left_front_child3_link": "fudan/r23_Link.STL",
    "Left_rear_link": "fudan/rf0_Link.STL",
    "Left_rear_child1_link": "fudan/rf1_Link.STL",
    "Left_Wheel_link": "ustl/Right_Wheel_link.STL",
    "Right_front_link": "fudan/l20_Link.STL",
    "Right_front_child1_link": "fudan/l21_Link.STL",
    "Right_front_child2_link": "fudan/l22_Link.STL",
    "Right_front_child3_link": "fudan/l23_Link.STL",
    "Right_rear_link": "fudan/lf0_Link.STL",
    "Right_rear_child1_link": "fudan/lf1_Link.STL",
    "Right_Wheel_link": "ustl/Left_Wheel_link.STL",
}

WHEEL_PARAMETERS = {
    "Left_Wheel_link": {
        "visual_quat": "0 0 0.707106781187 0.707106781187",
        "inertial_pos": "-0.00017762 -0.00010174 0.0099965",
        "inertial_quat": (
            "-0.511946370239 0.487829079253 "
            "0.511912146271 0.487728952933"
        ),
        "diaginertia": (
            "0.00119419026374 0.000629452355249 0.000625332573684"
        ),
        "collision_pos": "0 0 0.002",
    },
    "Right_Wheel_link": {
        "visual_quat": "0 0 0.707106781187 -0.707106781187",
        "inertial_pos": "-0.00015229 0.00013677 0.0099965",
        "inertial_quat": (
            "0.129231542436 0.695158797905 "
            "0.129317809464 -0.695219609088"
        ),
        "diaginertia": (
            "0.00119419026482 0.00062945508777 0.0006253298423"
        ),
        "collision_pos": "0 0 0.002",
    },
}

COLLIDING_LEG_BODIES = {
    "Left_front_link",
    "Left_front_child1_link",
    "Left_front_child2_link",
    "Left_front_child3_link",
    "Left_rear_link",
    "Left_rear_child1_link",
    "Right_front_link",
    "Right_front_child1_link",
    "Right_front_child2_link",
    "Right_front_child3_link",
    "Right_rear_link",
    "Right_rear_child1_link",
}

COLLISION_GEOM_NAMES = {
    "Left_rear_link": "rf0_collision",
    "Right_rear_link": "lf0_collision",
    "Left_front_child2_link": "r22_collision",
    "Right_front_child2_link": "l22_collision",
}

REBASE_QUATERNION = (0.0, 0.0, 0.0, 1.0)


def format_values(values: tuple[float, ...]) -> str:
    return " ".join(
        "0" if abs(value) < 1.0e-15 else f"{value:.15g}"
        for value in values
    )


def multiply_quaternions(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    result = (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )
    norm = math.sqrt(sum(value * value for value in result))
    return tuple(value / norm for value in result)


def rebase_pose(node: ET.Element) -> None:
    if node.get("pos") is not None:
        x, y, z = (float(value) for value in node.get("pos").split())
        node.set("pos", format_values((-x, -y, z)))
    quaternion = tuple(float(value) for value in node.get(
        "quat", "1 0 0 0").split())
    node.set("quat", format_values(multiply_quaternions(
        REBASE_QUATERNION, quaternion)))


def rebase_robot_contents(base: ET.Element) -> None:
    for child in base:
        if child.tag in {"body", "geom", "site", "camera", "light"}:
            rebase_pose(child)
        elif child.tag == "inertial":
            if child.get("pos") is not None:
                x, y, z = (
                    float(value) for value in child.get("pos").split())
                child.set("pos", format_values((-x, -y, z)))
            if child.get("quat") is not None:
                quaternion = tuple(
                    float(value) for value in child.get("quat").split())
                child.set("quat", format_values(multiply_quaternions(
                    REBASE_QUATERNION, quaternion)))
            if child.get("fullinertia") is not None:
                ixx, iyy, izz, ixy, ixz, iyz = (
                    float(value)
                    for value in child.get("fullinertia").split()
                )
                child.set("fullinertia", format_values((
                    ixx, iyy, izz, ixy, -ixz, -iyz,
                )))

def sync_assets() -> None:
    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    for target_path in set(MESH_FILES.values()):
        if Path(target_path).parts[0] != "fudan":
            continue
        target_name = Path(target_path).name
        source_name = (
            "base_link_del.STL"
            if target_name == "fudan_base_link_del.STL"
            else target_name
        )
        shutil.copyfile(
            SOURCE_ASSET_DIR / source_name,
            ASSET_DIR / target_name,
        )
    build_collision_proxy_assets(ASSET_DIR)


def rename_robot_tree(element: ET.Element) -> ET.Element:
    result = copy.deepcopy(element)

    def visit(node: ET.Element, body_name: str | None = None) -> None:
        if node.tag == "body":
            body_name = BODY_NAMES.get(node.get("name"), node.get("name"))
            node.set("name", body_name)
        elif node.tag == "joint":
            joint_name = JOINT_NAMES.get(node.get("name"), node.get("name"))
            node.set("name", joint_name)
        elif node.tag == "site":
            site_name = SITE_NAMES.get(node.get("name"), node.get("name"))
            node.set("name", site_name)
            if site_name in {
                "Left_front_site1", "Left_front_site2",
                "Right_front_site1", "Right_front_site2",
            }:
                position = [float(value) for value in node.get("pos").split()]
                position[2] = -0.01
                node.set("pos", " ".join(str(value) for value in position))
        elif node.tag == "geom":
            mesh_name = MESH_FILES.get(body_name)
            if mesh_name is not None:
                node.set("mesh", body_name)
            if body_name == "base_link":
                node.set("name", "base_link_collision")
                node.attrib.pop("condim", None)
                node.attrib.pop("friction", None)
                node.set("conaffinity", "1")
            elif body_name in COLLIDING_LEG_BODIES:
                node.set("conaffinity", "1")
                collision_name = COLLISION_GEOM_NAMES.get(body_name)
                if collision_name is not None:
                    node.set("name", collision_name)
            elif body_name in ("Left_Wheel_link", "Right_Wheel_link"):
                node.set("name", f"{body_name}_visual")
                node.set("type", "mesh")
                node.set("mesh", body_name)
                node.set("quat", WHEEL_PARAMETERS[body_name]["visual_quat"])
                node.set("contype", "0")
                node.set("conaffinity", "0")
        for child in node:
            visit(child, body_name)

    visit(result)
    return result


def add_assets(root: ET.Element) -> None:
    asset = root.find("asset")
    if asset is None:
        raise RuntimeError("plant shell is missing <asset>")
    for mesh in list(asset.findall("mesh")):
        if mesh.get("file") is not None:
            asset.remove(mesh)
    for name, file_name in MESH_FILES.items():
        ET.SubElement(asset, "mesh", {
            "name": name,
            "content_type": "model/stl",
            "file": file_name,
        })
    for spec in PROXY_SPECS:
        for sector in spec.sectors:
            ET.SubElement(asset, "mesh", {
                "name": proxy_mesh_name(spec, sector),
                "content_type": "model/stl",
                "file": f"fudan/{proxy_file_name(spec, sector)}",
            })


def replace_robot(root: ET.Element, fudan_root: ET.Element) -> None:
    worldbody = root.find("worldbody")
    source_worldbody = fudan_root.find("worldbody")
    if worldbody is None or source_worldbody is None:
        raise RuntimeError("missing worldbody")
    old_base = worldbody.find("body[@name='base_link']")
    source_base = source_worldbody.find("body[@name='base_Link_del']")
    if old_base is None or source_base is None:
        raise RuntimeError("missing robot base body")

    base = rename_robot_tree(source_base)
    rebase_robot_contents(base)
    base.set("name", "base_link")
    base.set("pos", "0 0 0.07")
    base.set("quat", "1 0 0 0")
    freejoint = base.find("freejoint")
    if freejoint is None:
        raise RuntimeError("Fudan base is missing freejoint")
    freejoint.set("name", "base_free_joint")

    base_geom = base.find("geom")
    if base_geom is None:
        raise RuntimeError("Fudan base is missing collision geom")
    base_geom.set("name", "base_link_collision")
    base_geom.set("mesh", "base_link")

    for body_name, site_name in (
        ("Left_Wheel_link", "Left_wheel_axis_site"),
        ("Right_Wheel_link", "Right_wheel_axis_site"),
    ):
        wheel_body = base.find(f".//body[@name='{body_name}']")
        if wheel_body is None:
            raise RuntimeError(f"missing wheel body {body_name}")
        parameters = WHEEL_PARAMETERS[body_name]
        inertial = wheel_body.find("inertial")
        if inertial is None:
            raise RuntimeError(f"missing wheel inertial {body_name}")
        inertial.set("pos", parameters["inertial_pos"])
        inertial.set("quat", parameters["inertial_quat"])
        inertial.set("mass", "0.71")
        inertial.set("diaginertia", parameters["diaginertia"])
        inertial.attrib.pop("fullinertia", None)
        ET.SubElement(wheel_body, "geom", {
            "name": (
                "Left_wheel_collision"
                if body_name == "Left_Wheel_link"
                else "Right_wheel_collision"
            ),
            "type": "cylinder",
            "pos": parameters["collision_pos"],
            "size": "0.058 0.028",
            "rgba": "0 0 0 0",
            "contype": "0",
            "conaffinity": "1",
            "friction": "1 0.005 0.0001",
        })
        ET.SubElement(wheel_body, "site", {
            "name": site_name,
            "pos": "0 0 0",
            "size": "0.005",
            "rgba": "0 0 1 1",
        })

    for spec in PROXY_SPECS:
        body = base.find(f".//body[@name='{spec.body_name}']")
        if body is None:
            raise RuntimeError(f"missing collision proxy body {spec.body_name}")
        insert_at = max(
            index for index, child in enumerate(body) if child.tag == "geom"
        ) + 1
        for sector in spec.sectors:
            name = proxy_mesh_name(spec, sector)
            geom = ET.Element("geom", {
                "name": name,
                "type": "mesh",
                "mesh": name,
                "contype": str(spec.contype),
                "conaffinity": str(spec.conaffinity),
                "condim": "1",
                "friction": "0 0 0",
                "group": "3",
                "rgba": "1 0.15 0.05 0.3",
            })
            body.insert(insert_at, geom)
            insert_at += 1

    for site in list(base.findall("site")):
        if site.get("name") != "imu_site":
            base.remove(site)
    imu_site = base.find("site[@name='imu_site']")
    if imu_site is None:
        raise RuntimeError("Fudan base is missing IMU site")
    ET.SubElement(base, "site", {
        "name": "Left_virtual_hip_site",
        "pos": "0 0.1877 0.08725",
        "size": "0.005",
        "rgba": "0 1 0 1",
    })
    ET.SubElement(base, "site", {
        "name": "Right_virtual_hip_site",
        "pos": "0 -0.1877 0.08725",
        "size": "0.005",
        "rgba": "0 1 0 1",
    })
    base_index = list(worldbody).index(old_base)
    worldbody[base_index] = base
    worldbody.insert(base_index, ET.Comment(
        " Keep the source model's 0.5 m body-to-floor placement relative "
        "to the\n         shell floor at z=-0.43 m; this lets the wheels "
        "touch down before the\n         base collision during the required "
        "free-release phase. "))


def configure_closed_chain(root: ET.Element) -> None:
    equality = root.find("equality")
    if equality is None:
        raise RuntimeError("plant shell is missing <equality>")
    for name in ("Left_loop1", "Left_loop2", "Right_loop1", "Right_loop2"):
        constraint = equality.find(f"connect[@name='{name}']")
        if constraint is None:
            raise RuntimeError(f"plant shell is missing equality {name}")
        constraint.set("solref", "0.00005 1")
        constraint.set("solimp", "0.99995 0.9999 0.00001 0.5 2")


def write_xml(root: ET.Element) -> None:
    ET.indent(root, space="  ")
    tree = ET.ElementTree(root)
    tree.write(OUTPUT, encoding="utf-8", xml_declaration=True)
    with OUTPUT.open("ab") as output:
        output.write(b"\n")


def main() -> None:
    root = ET.parse(COD_MODEL).getroot()
    fudan_root = ET.parse(FUDAN_MODEL).getroot()
    root.set("model", "Fudan-2026-Balance")
    root.find("compiler").set("angle", "radian")
    default_geom = root.find("default/geom")
    if default_geom is None:
        raise RuntimeError("plant shell is missing <default><geom>")
    default_geom.set("contype", "0")
    default_geom.set("conaffinity", "0")
    sync_assets()
    add_assets(root)
    replace_robot(root, fudan_root)
    configure_closed_chain(root)
    keyframe = root.find("keyframe")
    if keyframe is not None:
        root.remove(keyframe)
    write_xml(root)


if __name__ == "__main__":
    main()
