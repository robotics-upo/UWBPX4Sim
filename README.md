# UWBPX4Sim

A ready-to-use Ultra-Wideband simulator for relative localization of multi-robot teams integrated with ROS 2, Gazebo Harmonic and PX4 SITL.

`UWBPX4Sim` contains the Gazebo / PX4 assets used to simulate UWB ranging in the `mr-radio-localization` workspace.
It includes:

- modified Gazebo models for the `x500` UAV and the `r1_rover` UGV with onboard UWB tags / anchors
- the custom Gazebo system plugin `UWBGazeboPlugin`
- the `ROS2/px4_sim_offboard` ROS 2 package used to bridge Gazebo/PX4 topics and run the control-side nodes
- the `simulator_launcher.sh` tmux launcher used for PX4 SITL sessions
- world files and helper scripts for generating layouts and bridge configurations
- the generated `ROS2/px4_sim_offboard/config/uwb_bridge.yaml` GZ-to-ROS 2 bridge configuration

## Repository layout

- `models/base_models/`: template UAV and UGV models used as the generation inputs
- `models/custom_models/`: generated per-robot UAV and UGV models
- `ROS2/`: ROS 2 assets directory
- `ROS2/px4_sim_offboard/`: ROS 2 package for the PX4/Gazebo bridge and offboard-side nodes
- `ROS2/px4_sim_offboard/config/uwb_bridge.yaml`: generated GZ-to-ROS 2 bridge topics for all anchor-tag pairs
- `config/`: layout YAML files used for model generation and simulator spawning
- `simulator_launcher.sh`: tmux-based PX4 SITL launcher
- `uwb_gazebo_plugin/`: custom Gazebo system plugin source
- `worlds/`: Gazebo worlds used in the experiments
- `tools/`: helper scripts for sensor layout and mesh generation
- `config/uwb_layout.example.yaml`: example anchor / tag layout description

![](images/SimDiagram.png)

## What the plugin does

For each UGV anchor and UAV tag pair, the plugin publishes:

- a ground-truth range topic
- a simulated range topic with dropout and additive noise

The simulated model includes:

- LOS degradation as a function of distance
- NLOS degradation as a function of effective blocked thickness along the line of sight
- optional persistent pair bias
- one-sided stochastic NLOS noise, so NLOS tends to overestimate range rather than underestimate it
- body-shadow contribution from the robot bodies, capped so it cannot by itself exceed Soft NLOS

The default Gazebo topic families are:

- `/uwb_gz_simulator/distances/aItJ`
- `/uwb_gz_simulator/distances_ground_truth/aItJ`

where `aItJ` denotes anchor `I` and tag `J`.

## Measurement model summary

The plugin computes an effective blocked thickness for each anchor-tag line of sight:

- wall blockage is measured directly from world obstacle intersections
- robot-body blockage is converted to an equivalent thickness term
- body-shadow contribution is ignored below the LOS thickness threshold
- body-shadow contribution is capped so it cannot exceed the Soft-NLOS thickness ceiling by itself

The NLOS model lets the user tune:

- LOS / Soft NLOS / Hard NLOS / blackout boundaries
- dropout range and standard-deviation range
- how much faster Hard NLOS degrades than Soft NLOS
- how much more important thickness is than distance in NLOS

## Regenerating the UWB layout

From the repository root:

```bash
python3 tools/configure_uwb_layout.py --layout config/uwb_layout.example.yaml
```

This generates:

- one rover model per UGV, under `models/custom_models/r1_rover_<id>/`
- one UAV model per UAV, under `models/custom_models/x500_<id>/`
- `ROS2/px4_sim_offboard/config/uwb_bridge.yaml`

The layout YAML is structured robot-by-robot. Each vehicle entry can also include a `spawn_pose` 6-vector in world coordinates `[x, y, z, roll, pitch, yaw]`, which is used by the simulator launcher when spawning the robots. For example, the following four-robot layout assigns the same per-vehicle sensor geometry to two UAVs and two UGVs, while keeping anchor and tag ids globally unique:

```yaml
uavs:
  - id: 0
    spawn_pose: [3.0, 0.0, 0.0, 0.0, 0.0, 1.57079632679]
    tags:
      - id: 1
        position: [-0.24, -0.24, -0.06]
      - id: 2
        position: [0.24, 0.24, -0.06]

  - id: 1
    spawn_pose: [3.0, -5.0, 0.0, 0.0, 0.0, 1.57079632679]
    tags:
      - id: 3
        position: [-0.24, -0.24, -0.06]
      - id: 4
        position: [0.24, 0.24, -0.06]

ugvs:
  - id: 0
    spawn_pose: [0.0, 0.0, 0.0, 0.0, 0.0, 1.57079632679]
    anchors:
      - id: 1
        position: [-0.32, 0.3, 0.875]
      - id: 2
        position: [0.32, -0.3, 0.875]
      - id: 3
        position: [0.32, 0.3, 0.33]
      - id: 4
        position: [-0.32, -0.3, 0.33]

  - id: 1
    spawn_pose: [0.0, -5.0, 0.0, 0.0, 0.0, 1.57079632679]
    anchors:
      - id: 5
        position: [-0.32, 0.3, 0.875]
      - id: 6
        position: [0.32, -0.3, 0.875]
      - id: 7
        position: [0.32, 0.3, 0.33]
      - id: 8
        position: [-0.32, -0.3, 0.33]
```

Rules:

- UAV and UGV ids are integers matching the Gazebo/PX4 model suffix (`x500_0`, `r1_rover_0`, ...).
- `spawn_pose` is expressed as `[x, y, z, roll, pitch, yaw]` in world coordinates and is used by `simulator_launcher.sh`.
- Anchor ids are globally unique across all UGVs.
- Tag ids are globally unique across all UAVs.
- Pair topics remain `aItJ`, so `a1t2` is globally unique by construction.

The repository also includes this exact test layout as:

```text
config/uwb_layout.four_vehicle_example.yaml
```

## Generating simplified anchor/tag meshes

```bash
python3 tools/generate_uwb_meshes.py
```

This regenerates the STL meshes used for the mounted UWB devices in the models.

## ROS 2 workspace note

The `ROS2/` folder contains both:

- `ROS2/px4_sim_offboard/`: the ROS 2 package
- `ROS2/px4_sim_offboard/config/uwb_bridge.yaml`: the bridge configuration generated from the current layout

`px4_sim_offboard` must be present in your ROS 2 workspace and built before you launch the ROS-side nodes. If you clone the full `UWBPX4Sim` repository into your workspace `src/` directory, `colcon` will find `ROS2/px4_sim_offboard` automatically.

Example:

```bash
cd <ros_ws>/src
git clone https://github.com/amartinezsilva/UWBPX4Sim.git

cd <ros_ws>
colcon build --packages-select px4_sim_offboard
source install/setup.bash
```

After that, ROS 2 should be able to discover:

- `px4_sim_offboard`
- `ros2 launch px4_sim_offboard offboard_launch.py`
- `ros2 launch px4_sim_offboard uwb_bridge_launch.py`

## Spawning from the layout YAML

`simulator_launcher.sh` reads the same layout YAML used for model generation and takes each robot `spawn_pose` from there. By default it uses:

```text
config/uwb_layout.four_vehicle_example.yaml
```

You can point it to a different layout file with:

```bash
export UWB_LAYOUT_FILE=/path/to/your_layout.yaml
./simulator_launcher.sh
```

## Using the plugin in PX4 SITL

1. Copy the generated model folders from `models/custom_models/` into:

```text
<PX4-Autopilot>/Tools/simulation/gz/models/
```

2. Copy `uwb_gazebo_plugin/` into:

```text
<PX4-Autopilot>/src/modules/simulation/gz_plugins/
```

3. Register the plugin in the PX4 top-level `CMakeLists.txt`:

```cmake
add_subdirectory(uwb_gazebo_plugin)
add_custom_target(px4_gz_plugins ALL DEPENDS OpticalFlowSystem MovingPlatformController TemplatePlugin GenericMotorModelPlugin BuoyancySystemPlugin SpacecraftThrusterModelPlugin UWBGazeboPlugin)
```

4. Add the plugin instance to:

```text
<PX4-Autopilot>/src/modules/simulation/gz_bridge/server.config
```

Example plugin block:

```xml
<plugin entity_name="*" entity_type="world" filename="libUWBGazeboPlugin.so" name="custom::UWBGazeboSystem">
  <topic>/uwb_gz_simulator/distances</topic>
  <ground_truth_topic>/uwb_gz_simulator/distances_ground_truth</ground_truth_topic>

  <gaussian_noise_mean_cm>0.0</gaussian_noise_mean_cm>

  <apply_pair_bias>false</apply_pair_bias>
  <bias_min_cm>-27.0</bias_min_cm>
  <bias_max_cm>20.0</bias_max_cm>

  <enable_nlos_dropout>true</enable_nlos_dropout>
  <nlos_endpoint_margin_m>0.02</nlos_endpoint_margin_m>

  <los_dropout_start_distance_m>10.0</los_dropout_start_distance_m>
  <los_dropout_end_distance_m>70.0</los_dropout_end_distance_m>
  <los_hard_dropout_distance_m>75.0</los_hard_dropout_distance_m>

  <los_max_thickness_m>0.10</los_max_thickness_m>
  <soft_nlos_max_thickness_m>0.50</soft_nlos_max_thickness_m>
  <blackout_thickness_m>1.0</blackout_thickness_m>
  <hard_nlos_gap_ratio>0.75</hard_nlos_gap_ratio>

  <min_dropout_probability>0.02</min_dropout_probability>
  <max_dropout_probability>0.90</max_dropout_probability>

  <min_noise_stddev_cm>12.0</min_noise_stddev_cm>
  <max_noise_stddev_cm>50.0</max_noise_stddev_cm>
  <los_stddev_start_distance_m>15.0</los_stddev_start_distance_m>
  <los_stddev_end_distance_m>75.0</los_stddev_end_distance_m>

  <nlos_dropout_thickness_extra_weight>0.5</nlos_dropout_thickness_extra_weight>
  <nlos_stddev_thickness_extra_weight>0.5</nlos_stddev_thickness_extra_weight>
</plugin>
```

5. Rebuild PX4:

```bash
cd <PX4-Autopilot>
make px4_sitl
```

## Plugin parameters

The plugin accepts the following parameters.

| Parameter | Meaning | Code default |
| --- | --- | --- |
| `topic` | Base topic for simulated ranges | `/uwb_gz_simulator/distances` |
| `ground_truth_topic` | Base topic for ground-truth ranges | `/uwb_gz_simulator/distances_ground_truth` |
| `gaussian_noise_mean_cm` | Mean additive offset in cm | `0.0` |
| `apply_pair_bias` | Enables persistent per-pair bias | `false` |
| `bias_min_cm` | Minimum persistent pair bias in cm | `-27.0` |
| `bias_max_cm` | Maximum persistent pair bias in cm | `20.0` |
| `enable_nlos_dropout` | Enables NLOS-aware degradation logic | `true` |
| `nlos_endpoint_margin_m` | Ignores tiny intersections near the line-of-sight endpoints | `0.02` |
| `los_dropout_start_distance_m` | LOS distance where dropout begins to rise | `10.0` |
| `los_dropout_end_distance_m` | LOS distance where modeled dropout reaches its maximum pre-blackout value | `70.0` |
| `los_hard_dropout_distance_m` | LOS distance above which complete blackout is enforced | `75.0` |
| `los_max_thickness_m` | Maximum blocked thickness still treated as LOS | `0.10` |
| `soft_nlos_max_thickness_m` | Upper blocked-thickness bound for Soft NLOS | `0.50` |
| `blackout_thickness_m` | Blocked-thickness threshold for complete blackout | `1.0` |
| `hard_nlos_gap_ratio` | Extra Hard-NLOS slope relative to Soft NLOS (`0 = same slope`, `1 = double slope`) | `0.50` |
| `min_dropout_probability` | Minimum dropout probability | `0.02` |
| `max_dropout_probability` | Maximum dropout probability before blackout | `0.90` |
| `min_noise_stddev_cm` | Minimum noise standard deviation in cm | `12.0` |
| `max_noise_stddev_cm` | Maximum noise standard deviation in cm | `50.0` |
| `los_stddev_start_distance_m` | LOS distance where the stddev begins to rise | `15.0` |
| `los_stddev_end_distance_m` | LOS distance where the stddev reaches its maximum | `75.0` |
| `nlos_dropout_thickness_extra_weight` | Extra thickness importance for dropout relative to the fixed distance baseline (`0 = equal`, `1 = twice as influential`) | `0.50` |
| `nlos_stddev_thickness_extra_weight` | Extra thickness importance for stddev relative to the fixed distance baseline (`0 = equal`, `1 = twice as influential`) | `0.50` |
