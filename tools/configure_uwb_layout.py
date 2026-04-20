#!/usr/bin/env python3

"""Configure UWB anchor/tag layouts and generate per-robot Gazebo models.

This script updates:
  - models/custom_models/x500_<uav_id>/model.sdf + model.config
  - models/custom_models/r1_rover_<ugv_id>/model.sdf + model.config
  - ROS2/px4_sim_offboard/config/uwb_bridge.yaml

Layout input supports the multi-robot schema:
{
  "uavs": [
    {
      "id": 0,
      "tags": [
        {"id": 1, "position": [x, y, z]},
        ...
      ]
    },
    ...
  ],
  "ugvs": [
    {
      "id": 0,
      "anchors": [
        {"id": 1, "position": [x, y, z]},
        ...
      ]
    },
    ...
  ]
}

Global tag and anchor ids must be unique. Pair topics remain globally unique as
`aItJ`, where `I` is the unique anchor id and `J` is the unique tag id.

Legacy layouts are still accepted:
{
  "anchors": [[x, y, z], ...],
  "tags": [[x, y, z], ...]
}

Legacy layouts are interpreted as a single `ugv_id = 0` and `uav_id = 0`, with
anchor/tag ids assigned sequentially from 1.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple


BASE_MODELS_DIRNAME = "base_models"
CUSTOM_MODELS_DIRNAME = "custom_models"


@dataclass(frozen=True)
class SensorPlacement:
    id: int
    position: Tuple[float, float, float]


@dataclass(frozen=True)
class VehicleLayout:
    id: int
    sensors: List[SensorPlacement]


@dataclass(frozen=True)
class UwbLayout:
    uavs: List[VehicleLayout]
    ugvs: List[VehicleLayout]

    @property
    def all_tags(self) -> List[SensorPlacement]:
        return [sensor for uav in self.uavs for sensor in uav.sensors]

    @property
    def all_anchors(self) -> List[SensorPlacement]:
        return [sensor for ugv in self.ugvs for sensor in ugv.sensors]


@dataclass(frozen=True)
class SensorVisualSpec:
    mesh_uri: str
    mesh_scale: Tuple[float, float, float]
    mesh_color: str
    collision_size: Tuple[float, float, float]
    bar_radius: float
    bar_color: str


@dataclass(frozen=True)
class BlockBounds:
    start: int
    end: int
    base_indent: str
    indent_step: str


def _to_int(value: object, field_name: str) -> int:
    if isinstance(value, bool):
        raise ValueError(f"{field_name} must be an integer, not a boolean.")
    try:
        parsed = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field_name} must be an integer.") from exc
    if parsed < 0:
        raise ValueError(f"{field_name} must be non-negative.")
    return parsed


def _to_xyz(raw: object, field_name: str) -> Tuple[float, float, float]:
    if not isinstance(raw, (list, tuple)) or len(raw) != 3:
        raise ValueError(f"{field_name} must be a 3-element array [x,y,z].")
    try:
        x = float(raw[0])
        y = float(raw[1])
        z = float(raw[2])
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field_name} contains non-numeric values.") from exc
    return (x, y, z)


def _parse_sensor_entries(
    raw_sensors: object,
    *,
    field_name: str,
    sensor_label: str,
    global_seen_ids: set[int],
) -> List[SensorPlacement]:
    if raw_sensors is None:
        return []
    if not isinstance(raw_sensors, list):
        raise ValueError(f"{field_name} must be a list.")

    sensors: List[SensorPlacement] = []
    for idx, raw_sensor in enumerate(raw_sensors):
        entry_name = f"{field_name}[{idx}]"
        if not isinstance(raw_sensor, dict):
            raise ValueError(f"{entry_name} must be an object with 'id' and 'position'.")

        if "id" not in raw_sensor or "position" not in raw_sensor:
            raise ValueError(f"{entry_name} must include 'id' and 'position'.")

        sensor_id = _to_int(raw_sensor["id"], f"{entry_name}.id")
        if sensor_id in global_seen_ids:
            raise ValueError(f"Duplicate {sensor_label} id {sensor_id} found in layout.")
        global_seen_ids.add(sensor_id)

        sensors.append(
            SensorPlacement(
                id=sensor_id,
                position=_to_xyz(raw_sensor["position"], f"{entry_name}.position"),
            )
        )

    sensors.sort(key=lambda sensor: sensor.id)
    return sensors


def _parse_vehicle_layouts(
    raw_vehicles: object,
    *,
    field_name: str,
    sensor_field: str,
    sensor_label: str,
    global_seen_sensor_ids: set[int],
) -> List[VehicleLayout]:
    if not isinstance(raw_vehicles, list):
        raise ValueError(f"{field_name} must be a list.")

    seen_vehicle_ids: set[int] = set()
    vehicles: List[VehicleLayout] = []
    for idx, raw_vehicle in enumerate(raw_vehicles):
        entry_name = f"{field_name}[{idx}]"
        if not isinstance(raw_vehicle, dict):
            raise ValueError(f"{entry_name} must be an object with 'id' and '{sensor_field}'.")

        if "id" not in raw_vehicle:
            raise ValueError(f"{entry_name} must include 'id'.")

        vehicle_id = _to_int(raw_vehicle["id"], f"{entry_name}.id")
        if vehicle_id in seen_vehicle_ids:
            raise ValueError(f"Duplicate {field_name[:-1]} id {vehicle_id} found in layout.")
        seen_vehicle_ids.add(vehicle_id)

        sensors = _parse_sensor_entries(
            raw_vehicle.get(sensor_field, []),
            field_name=f"{entry_name}.{sensor_field}",
            sensor_label=sensor_label,
            global_seen_ids=global_seen_sensor_ids,
        )
        vehicles.append(VehicleLayout(id=vehicle_id, sensors=sensors))

    vehicles.sort(key=lambda vehicle: vehicle.id)
    return vehicles


def _load_legacy_layout(data: dict) -> UwbLayout:
    anchors_raw = data.get("anchors")
    tags_raw = data.get("tags")
    if anchors_raw is None or tags_raw is None:
        raise ValueError("Layout JSON must include either 'uavs'/'ugvs' or legacy 'anchors'/'tags'.")
    if not isinstance(anchors_raw, list) or not isinstance(tags_raw, list):
        raise ValueError("Legacy 'anchors' and 'tags' fields must be lists.")

    anchors = [
        SensorPlacement(id=idx, position=_to_xyz(raw_point, f"anchors[{idx - 1}]"))
        for idx, raw_point in enumerate(anchors_raw, start=1)
    ]
    tags = [
        SensorPlacement(id=idx, position=_to_xyz(raw_point, f"tags[{idx - 1}]"))
        for idx, raw_point in enumerate(tags_raw, start=1)
    ]

    return UwbLayout(
        uavs=[VehicleLayout(id=0, sensors=tags)],
        ugvs=[VehicleLayout(id=0, sensors=anchors)],
    )


def load_layout(layout_path: Path) -> UwbLayout:
    data = json.loads(layout_path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("Layout JSON must be an object.")

    if "uavs" in data or "ugvs" in data:
        if "uavs" not in data or "ugvs" not in data:
            raise ValueError("Multi-robot layouts must include both 'uavs' and 'ugvs'.")

        seen_tag_ids: set[int] = set()
        seen_anchor_ids: set[int] = set()

        uavs = _parse_vehicle_layouts(
            data["uavs"],
            field_name="uavs",
            sensor_field="tags",
            sensor_label="tag",
            global_seen_sensor_ids=seen_tag_ids,
        )
        ugvs = _parse_vehicle_layouts(
            data["ugvs"],
            field_name="ugvs",
            sensor_field="anchors",
            sensor_label="anchor",
            global_seen_sensor_ids=seen_anchor_ids,
        )
        layout = UwbLayout(uavs=uavs, ugvs=ugvs)
    else:
        layout = _load_legacy_layout(data)

    if not layout.all_anchors:
        raise ValueError("Layout must include at least one anchor.")
    if not layout.all_tags:
        raise ValueError("Layout must include at least one tag.")

    return layout


def _find_block_bounds(text: str, prefix: str) -> BlockBounds:
    first_link = re.search(rf'^(?P<indent>[ \t]*)<link name="{re.escape(prefix)}_\d+">', text, flags=re.M)
    if not first_link:
        raise RuntimeError(f"Could not find first link for prefix '{prefix}'.")

    first_link_start = first_link.start()
    base_indent = first_link.group("indent")

    joint_pattern = re.compile(
        rf'^[ \t]*<joint name="{re.escape(prefix)}_\d+_joint" type="fixed">[\s\S]*?^[ \t]*</joint>\n?',
        flags=re.M,
    )
    joint_matches = list(joint_pattern.finditer(text))
    if not joint_matches:
        raise RuntimeError(f"Could not find any fixed joints for prefix '{prefix}'.")

    end = joint_matches[-1].end()

    link_head_pattern = re.compile(
        rf'^(?P<link_indent>[ \t]*)<link name="{re.escape(prefix)}_\d+">\n(?P<inner_indent>[ \t]+)<pose>',
        flags=re.M,
    )
    head = link_head_pattern.search(text, pos=first_link_start)
    if head and head.group("inner_indent").startswith(head.group("link_indent")):
        indent_step = head.group("inner_indent")[len(head.group("link_indent")) :]
        if not indent_step:
            indent_step = "  "
    else:
        indent_step = "\t" if "\t" in base_indent else "  "

    return BlockBounds(start=first_link_start, end=end, base_indent=base_indent, indent_step=indent_step)


def _fmt_pose(x: float, y: float, z: float) -> str:
    return f"{x:g} {y:g} {z:g} 0 0 0"


def _rotation_matrix_from_axis_angle(
    axis: Tuple[float, float, float], angle: float
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]:
    ax, ay, az = axis
    c = math.cos(angle)
    s = math.sin(angle)
    one_c = 1.0 - c
    return (
        (c + ax * ax * one_c, ax * ay * one_c - az * s, ax * az * one_c + ay * s),
        (ay * ax * one_c + az * s, c + ay * ay * one_c, ay * az * one_c - ax * s),
        (az * ax * one_c - ay * s, az * ay * one_c + ax * s, c + az * az * one_c),
    )


def _rpy_from_rotation_matrix(
    rot: Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]
) -> Tuple[float, float, float]:
    r00, _r01, _r02 = rot[0]
    r10, _r11, _r12 = rot[1]
    r20, r21, r22 = rot[2]
    pitch = math.asin(max(-1.0, min(1.0, -r20)))
    roll = math.atan2(r21, r22)
    yaw = math.atan2(r10, r00)
    return roll, pitch, yaw


def _rotation_rpy_from_z_to_vector(vec: Tuple[float, float, float]) -> Tuple[float, float, float]:
    vx, vy, vz = vec
    norm = math.sqrt(vx * vx + vy * vy + vz * vz)
    if norm < 1e-9:
        return 0.0, 0.0, 0.0

    ux, uy, uz = vx / norm, vy / norm, vz / norm
    axis_x, axis_y, axis_z = -uy, ux, 0.0
    axis_norm = math.sqrt(axis_x * axis_x + axis_y * axis_y + axis_z * axis_z)

    if axis_norm < 1e-9:
        if uz >= 0.0:
            return 0.0, 0.0, 0.0
        return math.pi, 0.0, 0.0

    axis = (axis_x / axis_norm, axis_y / axis_norm, axis_z / axis_norm)
    angle = math.acos(max(-1.0, min(1.0, uz)))
    rot = _rotation_matrix_from_axis_angle(axis, angle)
    return _rpy_from_rotation_matrix(rot)


def _gen_sensor_link_block(
    *,
    prefix: str,
    sensor: SensorPlacement,
    base_indent: str,
    step: str,
    spec: SensorVisualSpec,
    label: str,
) -> str:
    i1 = base_indent + step
    i2 = i1 + step
    i3 = i2 + step
    i4 = i3 + step

    pose = _fmt_pose(*sensor.position)
    sx, sy, sz = sensor.position
    bar_vec = (-sx, -sy, -sz)
    bar_len = math.sqrt(bar_vec[0] * bar_vec[0] + bar_vec[1] * bar_vec[1] + bar_vec[2] * bar_vec[2])
    bar_block = ""
    if bar_len > 1e-6:
        bar_roll, bar_pitch, bar_yaw = _rotation_rpy_from_z_to_vector(bar_vec)
        bar_block = (
            f"{i1}<visual name=\"{prefix}_{sensor.id}_bar_vis\">\n"
            f"{i2}<pose>{bar_vec[0] / 2.0:g} {bar_vec[1] / 2.0:g} {bar_vec[2] / 2.0:g} "
            f"{bar_roll:g} {bar_pitch:g} {bar_yaw:g}</pose>\n"
            f"{i2}<geometry>\n"
            f"{i3}<cylinder>\n"
            f"{i4}<radius>{spec.bar_radius:g}</radius>\n"
            f"{i4}<length>{bar_len:g}</length>\n"
            f"{i3}</cylinder>\n"
            f"{i2}</geometry>\n"
            f"{i2}<material>\n"
            f"{i3}<ambient>{spec.bar_color}</ambient>\n"
            f"{i3}<diffuse>{spec.bar_color}</diffuse>\n"
            f"{i2}</material>\n"
            f"{i1}</visual>\n"
        )

    return (
        f'{base_indent}<link name="{prefix}_{sensor.id}">\n'
        f"{i1}<pose>{pose}</pose> <!-- {label} position relative to base_link -->\n"
        f"{i1}<inertial>\n"
        f"{i2}<mass>0.0001</mass> <!-- Very small mass -->\n"
        f"{i2}<inertia>\n"
        f"{i3}<ixx>1e-6</ixx>\n"
        f"{i3}<iyy>1e-6</iyy>\n"
        f"{i3}<izz>1e-6</izz>\n"
        f"{i3}<ixy>0</ixy>\n"
        f"{i3}<ixz>0</ixz>\n"
        f"{i3}<iyz>0</iyz>\n"
        f"{i2}</inertia>\n"
        f"{i1}</inertial>\n"
        f"{i1}<visual name=\"{prefix}_{sensor.id}_mesh_vis\">\n"
        f"{i2}<pose>0 0 0 0 0 0</pose>\n"
        f"{i2}<geometry>\n"
        f"{i3}<mesh>\n"
        f"{i4}<uri>{spec.mesh_uri}</uri>\n"
        f"{i4}<scale>{spec.mesh_scale[0]:g} {spec.mesh_scale[1]:g} {spec.mesh_scale[2]:g}</scale>\n"
        f"{i3}</mesh>\n"
        f"{i2}</geometry>\n"
        f"{i2}<material>\n"
        f"{i3}<ambient>{spec.mesh_color}</ambient>\n"
        f"{i3}<diffuse>{spec.mesh_color}</diffuse>\n"
        f"{i2}</material>\n"
        f"{i1}</visual>\n"
        f"{bar_block}"
        f"{i1}<collision name=\"{prefix}_{sensor.id}_collision\">\n"
        f"{i2}<pose>0 0 0 0 0 0</pose>\n"
        f"{i2}<geometry>\n"
        f"{i3}<box>\n"
        f"{i4}<size>{spec.collision_size[0]:g} {spec.collision_size[1]:g} {spec.collision_size[2]:g}</size>\n"
        f"{i3}</box>\n"
        f"{i2}</geometry>\n"
        f"{i1}</collision>\n"
        f"{base_indent}</link>\n"
    )


def _gen_sensor_joint_block(prefix: str, sensor_id: int, base_indent: str, step: str) -> str:
    i1 = base_indent + step
    return (
        f'{base_indent}<joint name="{prefix}_{sensor_id}_joint" type="fixed">\n'
        f"{i1}<parent>base_link</parent>\n"
        f"{i1}<child>{prefix}_{sensor_id}</child>\n"
        f"{base_indent}</joint>\n"
    )


def _gen_sensor_block(
    *,
    prefix: str,
    sensors: Sequence[SensorPlacement],
    base_indent: str,
    step: str,
    spec: SensorVisualSpec,
    label: str,
) -> str:
    blocks: List[str] = []
    for sensor in sensors:
        blocks.append(
            _gen_sensor_link_block(
                prefix=prefix,
                sensor=sensor,
                base_indent=base_indent,
                step=step,
                spec=spec,
                label=label,
            )
        )
        blocks.append(_gen_sensor_joint_block(prefix, sensor.id, base_indent, step))
        blocks.append("")

    return "\n".join(blocks).rstrip() + ("\n" if blocks else "")


def update_model_file(
    model_path: Path,
    *,
    prefix: str,
    sensors: Sequence[SensorPlacement],
    spec: SensorVisualSpec,
    label: str,
) -> None:
    text = model_path.read_text(encoding="utf-8")
    bounds = _find_block_bounds(text, prefix)

    replacement = _gen_sensor_block(
        prefix=prefix,
        sensors=sensors,
        base_indent=bounds.base_indent,
        step=bounds.indent_step,
        spec=spec,
        label=label,
    )

    updated = text[: bounds.start] + replacement + text[bounds.end :]
    model_path.write_text(updated, encoding="utf-8")


def _rewrite_model_sdf_identity(model_sdf_path: Path, source_model_name: str, target_model_name: str) -> None:
    text = model_sdf_path.read_text(encoding="utf-8")
    updated, count = re.subn(
        r'(<model\s+name=["\'])([^"\']+)(["\'])',
        rf"\1{target_model_name}\3",
        text,
        count=1,
    )
    if count != 1:
        raise RuntimeError(f"Could not rewrite model name in {model_sdf_path}")
    updated = updated.replace(f"model://{source_model_name}/", f"model://{target_model_name}/")
    model_sdf_path.write_text(updated, encoding="utf-8")


def _rewrite_model_config_identity(model_config_path: Path, target_model_name: str) -> None:
    text = model_config_path.read_text(encoding="utf-8")
    updated, count = re.subn(
        r"(<model>\s*<name>)([^<]+)(</name>)",
        rf"\1{target_model_name}\3",
        text,
        count=1,
        flags=re.S,
    )
    if count != 1:
        raise RuntimeError(f"Could not rewrite model.config name in {model_config_path}")
    model_config_path.write_text(updated, encoding="utf-8")


def _copy_and_prepare_model_dir(
    *,
    template_dir: Path,
    target_dir: Path,
    target_model_name: str,
) -> Path:
    if target_dir.exists():
        shutil.rmtree(target_dir)
    shutil.copytree(template_dir, target_dir)

    model_sdf = target_dir / "model.sdf"
    model_config = target_dir / "model.config"
    _rewrite_model_sdf_identity(model_sdf, template_dir.name, target_model_name)
    _rewrite_model_config_identity(model_config, target_model_name)
    return model_sdf


def _tag_visual_spec(model_name: str) -> SensorVisualSpec:
    return SensorVisualSpec(
        mesh_uri=f"model://{model_name}/meshes/uwb_tag_eliko.stl",
        mesh_scale=(1.0, 1.0, 1.0),
        mesh_color="0.96 0.55 0.13 1",
        collision_size=(0.052, 0.036, 0.030),
        bar_radius=0.0035,
        bar_color="0.24 0.24 0.24 1",
    )


def _anchor_visual_spec(model_name: str) -> SensorVisualSpec:
    return SensorVisualSpec(
        mesh_uri=f"model://{model_name}/meshes/uwb_anchor_eliko.stl",
        mesh_scale=(1.0, 1.0, 1.0),
        mesh_color="0.92 0.92 0.92 1",
        collision_size=(0.062, 0.046, 0.040),
        bar_radius=0.004,
        bar_color="0.28 0.28 0.30 1",
    )


def generate_bridge_yaml(layout: UwbLayout) -> str:
    lines: List[str] = []
    lines.append("## ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=uwb_bridge.yaml")
    lines.append("")
    lines.append("# UWB distance topic bridges (GZ_TO_ROS only)")

    def append_bridge(topic: str) -> None:
        lines.extend(
            [
                f'- ros_topic_name: "{topic}"',
                f'  gz_topic_name: "{topic}"',
                '  ros_type_name: "std_msgs/msg/Float64"',
                '  gz_type_name: "gz.msgs.Double"',
                "  subscriber_queue: 10",
                "  publisher_queue: 10",
                "  lazy: false",
                "  direction: GZ_TO_ROS",
                "",
            ]
        )

    anchor_ids = sorted(sensor.id for sensor in layout.all_anchors)
    tag_ids = sorted(sensor.id for sensor in layout.all_tags)

    for anchor_id in anchor_ids:
        for tag_id in tag_ids:
            append_bridge(f"/uwb_gz_simulator/distances/a{anchor_id}t{tag_id}")

    lines.append("# UWB ground-truth distance topic bridges (GZ_TO_ROS only)")
    lines.append("")

    for anchor_id in anchor_ids:
        for tag_id in tag_ids:
            append_bridge(f"/uwb_gz_simulator/distances_ground_truth/a{anchor_id}t{tag_id}")

    while lines and lines[-1] == "":
        lines.pop()
    lines.append("")
    return "\n".join(lines)


def _find_offboard_bridge_targets(uwb_root: Path) -> List[Path]:
    candidates = [uwb_root / "ROS2" / "px4_sim_offboard" / "config" / "uwb_bridge.yaml"]
    targets: List[Path] = []
    seen: set[Path] = set()
    for candidate in candidates:
        resolved = candidate.resolve()
        if candidate.parent.exists() and resolved not in seen:
            seen.add(resolved)
            targets.append(candidate)
    return targets


def _format_generated_model_paths(model_dirs: Iterable[Path]) -> str:
    sorted_dirs = sorted(model_dirs)
    return "\n".join(f"  - {path}" for path in sorted_dirs)


def main() -> None:
    parser = argparse.ArgumentParser(description="Configure UWB layouts and generate per-robot Gazebo models.")
    parser.add_argument(
        "--layout",
        required=True,
        type=Path,
        help="Path to layout JSON file. Supports multi-robot and legacy schemas.",
    )
    parser.add_argument(
        "--uwb-root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
        help="Path to UWBPX4Sim directory (default: script's parent).",
    )
    parser.add_argument(
        "--skip-bridge",
        action="store_true",
        help="Do not regenerate the GZ-to-ROS2 bridge YAML files.",
    )
    args = parser.parse_args()

    layout = load_layout(args.layout)
    uwb_root = args.uwb_root.resolve()

    models_dir = uwb_root / "models"
    base_models_dir = models_dir / BASE_MODELS_DIRNAME
    custom_models_dir = models_dir / CUSTOM_MODELS_DIRNAME
    uav_template_dir = base_models_dir / "x500_base"
    ugv_template_dir = base_models_dir / "r1_rover"
    bridge_yaml = uwb_root / "ROS2" / "px4_sim_offboard" / "config" / "uwb_bridge.yaml"

    for path in (uav_template_dir, ugv_template_dir, uav_template_dir / "model.sdf", ugv_template_dir / "model.sdf"):
        if not path.exists():
            raise FileNotFoundError(f"Required template path not found: {path}")
    if not args.skip_bridge and not bridge_yaml.parent.exists():
        raise FileNotFoundError(f"Bridge config directory not found: {bridge_yaml.parent}")

    custom_models_dir.mkdir(parents=True, exist_ok=True)

    generated_model_dirs: List[Path] = []

    for uav in layout.uavs:
        model_name = f"x500_{uav.id}"
        target_dir = custom_models_dir / model_name
        model_sdf = _copy_and_prepare_model_dir(
            template_dir=uav_template_dir,
            target_dir=target_dir,
            target_model_name=model_name,
        )
        update_model_file(
            model_sdf,
            prefix="uwb_tag",
            sensors=uav.sensors,
            spec=_tag_visual_spec(model_name),
            label="Tag",
        )
        generated_model_dirs.append(target_dir)

    for ugv in layout.ugvs:
        model_name = f"r1_rover_{ugv.id}"
        target_dir = custom_models_dir / model_name
        model_sdf = _copy_and_prepare_model_dir(
            template_dir=ugv_template_dir,
            target_dir=target_dir,
            target_model_name=model_name,
        )
        update_model_file(
            model_sdf,
            prefix="uwb_anchor",
            sensors=ugv.sensors,
            spec=_anchor_visual_spec(model_name),
            label="Anchor",
        )
        generated_model_dirs.append(target_dir)

    bridge_targets: List[Path] = []
    if not args.skip_bridge:
        bridge_text = generate_bridge_yaml(layout)
        bridge_yaml.write_text(bridge_text, encoding="utf-8")
        bridge_targets.append(bridge_yaml)
        seen_bridge_targets = {bridge_yaml.resolve()}

        for offboard_bridge_yaml in _find_offboard_bridge_targets(uwb_root):
            resolved_target = offboard_bridge_yaml.resolve()
            if resolved_target in seen_bridge_targets:
                continue
            offboard_bridge_yaml.write_text(bridge_text, encoding="utf-8")
            bridge_targets.append(offboard_bridge_yaml)
            seen_bridge_targets.add(resolved_target)

    print(
        "Updated UWB layout:\n"
        f"  - {len(layout.ugvs)} UGV model(s)\n"
        f"  - {len(layout.uavs)} UAV model(s)\n"
        f"  - {len(layout.all_anchors)} anchor(s)\n"
        f"  - {len(layout.all_tags)} tag(s)\n"
        "Generated models:\n"
        f"{_format_generated_model_paths(generated_model_dirs)}"
        + (
            "\nBridge files:\n" + "\n".join(f"  - {path}" for path in bridge_targets)
            if bridge_targets
            else ""
        )
    )


if __name__ == "__main__":
    main()
