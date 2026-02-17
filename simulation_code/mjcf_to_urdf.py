#!/usr/bin/env python3
"""
mjcf_to_urdf.py
===============
Converts the SO101 MuJoCo MJCF model to a URDF/XACRO file suitable for
Gazebo Harmonic + ros2_control.

The converter:
  1. Loads the MuJoCo model via the Python API.
  2. Reads each body's position (m.body_pos) and quaternion (m.body_quat)
     in the PARENT-BODY frame (the values stored in the XML).
  3. Converts quaternions to roll-pitch-yaw (ZYX Euler) using scipy.
  4. Reads joint data: axis, limits, damping.
  5. Writes a valid URDF with <ros2_control> and Gazebo plugin tags.

Usage (from simulation_code/):
    python mjcf_to_urdf.py
    → writes  model/so101.urdf.xacro
              model/so101.urdf          (plain URDF for quick inspection)

Requirements:  pip install mujoco scipy
"""

from __future__ import annotations
import os
import sys
import xml.etree.ElementTree as ET
from textwrap import indent

import numpy as np
import mujoco

try:
    from scipy.spatial.transform import Rotation
    _SCIPY = True
except Exception:
    _SCIPY = False
    print("[warn] scipy unavailable – falling back to manual quaternion→rpy")


# ─────────────────────── CONFIG ────────────────────────────────────────────

MJCF_PATH  = "model/robot_from_urdf.xml"
OUT_XACRO  = "model/so101.urdf.xacro"
OUT_URDF   = "model/so101.urdf"

# Body chain (in order base→tip).  The root body is "base".
BODY_CHAIN = [
    "base",
    "shoulder",
    "upper_arm",
    "lower_arm",
    "wrist",
    "gripper",
    "moving_jaw_so101_v1",
]

# Joint names in order (each connects consecutive bodies above).
JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

# Map body → URDF link name (keeps names short and valid).
LINK_NAME = {
    "base":               "base_link",
    "shoulder":           "shoulder_link",
    "upper_arm":          "upper_arm_link",
    "lower_arm":          "lower_arm_link",
    "wrist":              "wrist_link",
    "gripper":            "gripper_link",
    "moving_jaw_so101_v1":"moving_jaw_link",
}

MESH_PACKAGE = "package://gazebo_mujoco_bridge/meshes"


# ─────────────────────── XML GEOM PARSER ─────────────────────────────────

def parse_xml_geoms(xml_path: str) -> dict:
    """Parse MJCF XML directly to extract geom placement data.

    This avoids MuJoCo's compiled model where mesh centering has already
    modified geom_pos/geom_quat.  The raw XML values position the mesh's
    native STL origin in the body frame — exactly what URDF needs.

    Returns
    -------
    dict : {body_name: [(mesh_name, pos_xyz, quat_wxyz, geom_class, rgba), ...]}
    """
    tree = ET.parse(xml_path)
    root = tree.getroot()

    # Parse materials from <asset>
    materials: dict[str, list[float]] = {}
    for asset in root.iter("asset"):
        for mat in asset.findall("material"):
            name = mat.get("name", "")
            rgba_str = mat.get("rgba", "1 1 1 1")
            materials[name] = [float(v) for v in rgba_str.split()]

    # Parse geoms from each body (only direct children, not nested bodies)
    body_geoms: dict[str, list] = {}
    for body in root.iter("body"):
        bname = body.get("name", "")
        geoms = []
        for geom in body.findall("geom"):
            if geom.get("type", "") != "mesh":
                continue
            mesh_name = geom.get("mesh", "")
            pos_str   = geom.get("pos", "0 0 0")
            quat_str  = geom.get("quat", "1 0 0 0")
            gclass    = geom.get("class", "")
            mat_name  = geom.get("material", "")

            pos  = np.array([float(v) for v in pos_str.split()])
            quat = np.array([float(v) for v in quat_str.split()])
            rgba = np.array(materials.get(mat_name, [1.0, 1.0, 1.0, 1.0]))

            geoms.append((mesh_name, pos, quat, gclass, rgba))
        if geoms:
            body_geoms[bname] = geoms

    return body_geoms


# ─────────────────────── QUATERNION → RPY ──────────────────────────────────

def quat_wxyz_to_rpy(quat_wxyz: np.ndarray) -> tuple[float, float, float]:
    """
    Convert MuJoCo quaternion [w,x,y,z] to URDF roll-pitch-yaw (ZYX Euler).
    """
    w, x, y, z = quat_wxyz
    if _SCIPY:
        r = Rotation.from_quat([x, y, z, w])          # scipy uses [x,y,z,w]
        roll, pitch, yaw = r.as_euler("xyz", degrees=False)
    else:
        # Manual ZYX from rotation matrix
        R = np.array([
            [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y)],
            [  2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x)],
            [  2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)],
        ])
        pitch = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))
        if abs(abs(pitch) - np.pi / 2) < 1e-6:          # gimbal lock
            roll = 0.0
            yaw  = np.arctan2(R[0, 1], R[1, 1])
        else:
            roll = np.arctan2(R[2, 1], R[2, 2])
            yaw  = np.arctan2(R[1, 0], R[0, 0])
    return float(roll), float(pitch), float(yaw)



# ─────────────────────── URDF BUILDER ──────────────────────────────────────

def _fmt(v: float) -> str:
    return f"{v:.8f}"


def _origin(xyz: np.ndarray, rpy: tuple[float, float, float]) -> str:
    x, y, z = xyz
    r, p, yw = rpy
    return (
        f'<origin xyz="{_fmt(x)} {_fmt(y)} {_fmt(z)}" '
        f'rpy="{_fmt(r)} {_fmt(p)} {_fmt(yw)}"/>'
    )


def _inertial(mass: float, inertia_6: np.ndarray, com: np.ndarray) -> str:
    ixx, iyy, izz, ixy, ixz, iyz = inertia_6
    cx, cy, cz = com
    return (
        "<inertial>\n"
        f'  <origin xyz="{_fmt(cx)} {_fmt(cy)} {_fmt(cz)}" rpy="0 0 0"/>\n'
        f'  <mass value="{_fmt(mass)}"/>\n'
        f'  <inertia ixx="{_fmt(ixx)}" ixy="{_fmt(ixy)}" ixz="{_fmt(ixz)}" '
        f'iyy="{_fmt(iyy)}" iyz="{_fmt(iyz)}" izz="{_fmt(izz)}"/>\n'
        "</inertial>"
    )


def parse_xml_inertias(xml_path: str) -> dict:
    """Parse fullinertia values from MJCF XML to preserve off-diagonal terms.

    Returns
    -------
    dict : {body_name: [ixx, iyy, izz, ixy, ixz, iyz]}
           Mapped to URDF ordering: ixx, iyy, izz, ixy, ixz, iyz.
    """
    tree = ET.parse(xml_path)
    root = tree.getroot()
    result = {}
    for body in root.iter("body"):
        bname = body.get("name", "")
        inertial = body.find("inertial")
        if inertial is not None:
            fi = inertial.get("fullinertia")
            if fi:
                vals = [float(v) for v in fi.split()]
                # MJCF fullinertia: ixx iyy izz ixy ixz iyz
                result[bname] = vals
    return result


def build_urdf(m: mujoco.MjModel, xml_geoms: dict, xml_inertias: dict | None = None) -> str:
    """Construct URDF XML string from the MuJoCo model.

    Parameters
    ----------
    m : MjModel  – compiled model (for joints, inertials, body transforms)
    xml_geoms    – output of parse_xml_geoms() (raw geom placement from XML)
    xml_inertias – output of parse_xml_inertias() (full 6-component inertia from XML)
    """

    lines: list[str] = []
    lines.append('<?xml version="1.0"?>')
    lines.append(
        '<robot name="so101"\n'
        '       xmlns:xacro="http://www.ros.org/wiki/xacro">'
    )

    # ── Gazebo native plugins (no gz_ros2_control needed) ────────────────
    lines.append("")
    lines.append("  <!-- ═══════════ Gazebo plugins ═══════════ -->")
    lines.append("  <!-- JointPositionController for each joint -->")
    for jn in JOINT_NAMES:
        lines.append("  <gazebo>")
        lines.append(
            "    <plugin filename=\"gz-sim-joint-position-controller-system\" "
            'name="gz::sim::systems::JointPositionController">'
        )
        lines.append(f"      <joint_name>{jn}</joint_name>")
        lines.append(f"      <topic>/model/so101/joint/{jn}/cmd_pos</topic>")
        # Use the same servo gains as MuJoCo (kp=998.22, kv=2.731)
        lines.append("      <p_gain>998.22</p_gain>")
        lines.append("      <d_gain>2.731</d_gain>")
        lines.append("      <i_gain>0.0</i_gain>")
        lines.append("    </plugin>")
        lines.append("  </gazebo>")

    # ── JointStatePublisher ────────────────────────────────────────────────
    lines.append("  <!-- JointStatePublisher: publishes on /world/<w>/model/so101/joint_state -->")
    lines.append("  <gazebo>")
    lines.append(
        "    <plugin filename=\"gz-sim-joint-state-publisher-system\" "
        'name="gz::sim::systems::JointStatePublisher"/>'
    )
    lines.append("  </gazebo>")

    # ── Links and joints ──────────────────────────────────────────────────
    lines.append("")
    lines.append("  <!-- ═══════════ Links & Joints ═══════════ -->")

    # Fixed joint anchoring the base to the world (prevents free-floating)
    lines.append('  <link name="world"/>')
    lines.append('  <joint name="base_fixed_to_world" type="fixed">')
    lines.append('    <parent link="world"/>')
    lines.append('    <child link="base_link"/>')
    lines.append('    <origin xyz="0 0 0" rpy="0 0 0"/>')
    lines.append('  </joint>')
    lines.append("")

    joint_idx = 0   # which joint we are about to emit

    for chain_idx, body_name in enumerate(BODY_CHAIN):
        link_name = LINK_NAME[body_name]
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if bid < 0:
            print(f"[warn] body '{body_name}' not found – skipping", file=sys.stderr)
            continue

        # Inertial from model
        mass    = float(m.body_mass[bid])
        com     = np.array(m.body_ipos[bid])          # CoM in body frame
        # Prefer full inertia tensor from XML (preserves off-diagonal terms)
        if xml_inertias and body_name in xml_inertias:
            fi = xml_inertias[body_name]
            # MJCF fullinertia: ixx iyy izz ixy ixz iyz → URDF: ixx iyy izz ixy ixz iyz
            inertia_6 = fi
        else:
            inertia = np.array(m.body_inertia[bid])   # [Ixx, Iyy, Izz] principal
            inertia_6 = [inertia[0], inertia[1], inertia[2], 0.0, 0.0, 0.0]

        # Collect mesh geoms from the XML-parsed data (avoids MuJoCo's
        # compiled mesh centering adjustment – XML values position the
        # mesh's native STL origin directly in the body frame).
        visuals = []    # (mesh_name, urdf_pos, urdf_rpy, rgba)
        collisions = []
        for mesh_name, gpos, gquat, gclass, rgba in xml_geoms.get(body_name, []):
            rpy = quat_wxyz_to_rpy(gquat)
            entry = (mesh_name, gpos, rpy, rgba)
            if gclass == "visual":
                visuals.append(entry)
            elif gclass == "collision":
                collisions.append(entry)

        lines.append(f'  <link name="{link_name}">')
        lines.append(
            indent(_inertial(mass, inertia_6, com), "    ")
        )

        # Emit visual elements from MuJoCo geoms
        for vi, (mesh_name, upos, urpy, rgba) in enumerate(visuals):
            vname = f"{link_name}_visual_{vi}"
            lines.append(f'    <visual name="{vname}">')
            lines.append(f"      {_origin(upos, urpy)}")
            lines.append("      <geometry>")
            lines.append(
                f'        <mesh filename="{MESH_PACKAGE}/{mesh_name}.stl"/>'
            )
            lines.append("      </geometry>")
            r, g, b, a = float(rgba[0]), float(rgba[1]), float(rgba[2]), float(rgba[3])
            lines.append(f'      <material name="{mesh_name}_mat">')
            lines.append(f'        <color rgba="{r:.3f} {g:.3f} {b:.3f} {a:.3f}"/>')
            lines.append("      </material>")
            lines.append("    </visual>")

        # Fallback: if body has visuals but no collision geoms, use visuals as collision
        if visuals and not collisions:
            collisions = visuals

        # Emit collision elements
        for ci, (mesh_name, upos, urpy, rgba) in enumerate(collisions):
            cname = f"{link_name}_collision_{ci}"
            lines.append(f'    <collision name="{cname}">')
            lines.append(f"      {_origin(upos, urpy)}")
            lines.append("      <geometry>")
            lines.append(
                f'        <mesh filename="{MESH_PACKAGE}/{mesh_name}.stl"/>'
            )
            lines.append("      </geometry>")
            lines.append("    </collision>")

        # Fallback: if no visuals found, add a tiny sphere so link is visible
        if not visuals:
            lines.append(f'    <visual name="{link_name}_visual_fallback">')
            lines.append('      <geometry><sphere radius="0.005"/></geometry>')
            lines.append("    </visual>")

        lines.append("  </link>")

        # ── Joint connecting parent → this body ───────────────────────────
        if chain_idx > 0:
            parent_link = LINK_NAME[BODY_CHAIN[chain_idx - 1]]

            # body_pos / body_quat in PARENT frame
            pos  = np.array(m.body_pos[bid])
            quat = np.array(m.body_quat[bid])   # [w, x, y, z]
            rpy  = quat_wxyz_to_rpy(quat)

            jn = JOINT_NAMES[joint_idx]
            joint_idx += 1

            # Joint limits from model
            jid      = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)
            jlim_lo  = float(m.jnt_range[jid, 0])
            jlim_hi  = float(m.jnt_range[jid, 1])
            # Axis in body frame (MuJoCo stores as unit vec)
            j_axis   = m.jnt_axis[jid]
            # Damping from body dof
            dof_adr  = int(m.jnt_dofadr[jid])
            damping  = float(m.dof_damping[dof_adr])
            friction = float(m.dof_frictionloss[dof_adr])

            lines.append(f'  <joint name="{jn}" type="revolute">')
            lines.append(f"    {_origin(pos, rpy)}")
            lines.append(f'    <parent link="{parent_link}"/>')
            lines.append(f'    <child link="{link_name}"/>')
            lines.append(
                f'    <axis xyz="{_fmt(j_axis[0])} {_fmt(j_axis[1])} {_fmt(j_axis[2])}"/>'
            )
            lines.append(
                f'    <limit lower="{_fmt(jlim_lo)}" upper="{_fmt(jlim_hi)}" '
                f'effort="3.35" velocity="10.0"/>'
            )
            lines.append(
                f'    <dynamics damping="{_fmt(damping)}" '
                f'friction="{_fmt(friction)}"/>'
            )
            lines.append("  </joint>")

    lines.append("")
    lines.append("</robot>")
    return "\n".join(lines)


def build_xacro_wrapper(urdf_content: str) -> str:
    """
    Wrap the bare URDF in a minimal XACRO file that also exposes
    a `mesh_dir` argument for overriding the STL path.
    """
    header = (
        '<?xml version="1.0"?>\n'
        '<robot name="so101" xmlns:xacro="http://www.ros.org/wiki/xacro">\n'
        "\n"
        "  <!-- Override mesh_dir to point to your STL assets -->\n"
        f'  <xacro:arg name="mesh_dir" default="{MESH_PACKAGE}"/>\n'
        "\n"
    )
    # Strip the XML declaration and <robot> wrapper from the bare URDF
    body = "\n".join(
        line for line in urdf_content.splitlines()
        if not line.startswith("<?xml")
        and not line.startswith("<robot")
        and not line.startswith("</robot")
        and not line.strip().startswith("xmlns:xacro=")
    )
    return header + body + "\n</robot>\n"


# ─────────────────────── ENTRY POINT ───────────────────────────────────────

def main():
    print(f"Loading model: {MJCF_PATH}")
    m = mujoco.MjModel.from_xml_path(MJCF_PATH)

    print("Parsing XML geom data (bypasses mesh centering) …")
    xml_geoms = parse_xml_geoms(MJCF_PATH)
    for bname, glist in xml_geoms.items():
        print(f"  {bname}: {len(glist)} geom(s)")

    print("Parsing XML inertia data (preserves off-diagonal terms) …")
    xml_inertias = parse_xml_inertias(MJCF_PATH)

    print("Building URDF …")
    urdf = build_urdf(m, xml_geoms, xml_inertias)

    with open(OUT_URDF, "w") as f:
        f.write(urdf)
    print(f"  Wrote: {OUT_URDF}")

    xacro = build_xacro_wrapper(urdf)
    with open(OUT_XACRO, "w") as f:
        f.write(xacro)
    print(f"  Wrote: {OUT_XACRO}")

    print("\nJoint summary:")
    for jn in JOINT_NAMES:
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)
        lo, hi = m.jnt_range[jid]
        bid = m.jnt_bodyid[jid]
        pos = m.body_pos[bid]
        quat = m.body_quat[bid]
        rpy = quat_wxyz_to_rpy(quat)
        print(
            f"  {jn:15s}  range=[{np.rad2deg(lo):.1f}°, {np.rad2deg(hi):.1f}°]  "
            f"body_pos={np.array2string(pos, precision=4)}  "
            f"rpy=[{', '.join(f'{np.rad2deg(v):.1f}°' for v in rpy)}]"
        )


if __name__ == "__main__":
    main()
