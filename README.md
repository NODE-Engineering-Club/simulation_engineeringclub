# asket_ec_sim2d — 2D Simulation for Asket EC

Lightweight pure-Python 2D simulation of the **Asket EC** autonomous surface vessel, built to run on a low-spec laptop (i5-8265U, 8 GB RAM, WSL2 Ubuntu 24.04, ROS2 Jazzy).

**Purpose:** validate the Njord PID control chain and gate navigation behaviour before deploying to the real boat. This is a physics engine and synthetic sensor data provider — not a Gazebo replacement and not a Nav2 replacement.

---

## What this is and what it is not

| This simulation… | …is | …is not |
|---|---|---|
| Physics engine | 50 Hz differential drive with viscous drag | A 3D simulator |
| Sensor model | Camera FOV + Gaussian noise, fake GPS | Real sensor drivers |
| Path planner | Pure pursuit through gate centres | Nav2 / global planner |
| Control chain | Njord `nav_to_pid` + `pid_controller` | Full Njord stack |
| Visualisation | RViz2 (boat shape, trajectory, buoys) | Gazebo rendering |

Gazebo Harmonic was replaced because it crashes on WSL2 with Intel integrated graphics (segfaults, OpenGL failures). This simulator starts in under one second and runs stably on the same hardware.

---

## Repository structure

```
simulation_engineeringclub/
├── README.md
└── asket_ec_sim_ws/
    └── src/
        └── asket_ec_sim2d/          ← only active package
            ├── asket_ec_sim2d/
            │   ├── simulator_node.py
            │   ├── buoy_simulator_node.py
            │   ├── waypoint_navigator_node.py
            │   ├── sim_bridge_node.py
            │   ├── nav_to_pid.py
            │   ├── pid_controller.py
            │   └── keyboard_teleop_node.py
            ├── config/
            │   ├── buoys.yaml        ← gate definitions (GPS)
            │   ├── waypoints.yaml    ← fallback waypoints (GPS)
            │   └── sim2d.rviz
            └── launch/
                ├── njord_sim.launch.py   ← full stack (use this)
                └── sim2d.launch.py       ← sim only, no PID chain
```

> `asket_ec_control`, `asket_ec_description`, `asket_ec_gazebo` are kept for reference only — they do not build or run on WSL2.

---

## Prerequisites

| Requirement | Version |
|---|---|
| Ubuntu | 24.04 LTS or WSL2 |
| ROS2 | Jazzy Jalisco |
| Python | 3.12 (included with Ubuntu 24.04) |

---

## One-time setup

```bash
git clone https://github.com/NODE-Engineering-Club/simulation_engineeringclub
cd simulation_engineeringclub/asket_ec_sim_ws

colcon build --packages-select asket_ec_sim2d
source install/setup.bash
```

### Known build issue — setuptools 68 / colcon conflict

On Ubuntu 24.04, setuptools 68 places executable scripts in `install/asket_ec_sim2d/bin/` instead of the `lib/asket_ec_sim2d/` directory that `ros2 launch` expects. Apply this symlink workaround after every clean rebuild:

```bash
mkdir -p install/asket_ec_sim2d/lib/asket_ec_sim2d
cd install/asket_ec_sim2d/lib/asket_ec_sim2d
for f in ../../bin/*; do ln -sf "$f" .; done
cd -
```

**Known fix (not yet applied):** downgrade setuptools to < 68 or migrate `package.xml` to the `ament_cmake_python` build type. Tracked as a known issue.

---

## Running the simulation

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch asket_ec_sim2d njord_sim.launch.py
```

RViz2 opens automatically with a pre-configured layout. The boat starts at the origin and navigates through the three buoy gates.

**What you should see:**
- Boat (white hull, orange bow) moving north through the gates
- Cyan line: trajectory history
- Red/green spheres: buoys (bright = in camera FOV, dim = out of range)
- Yellow line: planned route through gate centres
- Labels above visible buoys: `GATE X — RED / GREEN`

---

## Node architecture

```
buoy_simulator_node ──/gates/centers──▶ waypoint_navigator_node
                    ──/buoys/all──────▶ RViz2
                    ──/buoys/detected─▶ RViz2

waypoint_navigator_node ──/nav/cmd_vel──▶ nav_to_pid

nav_to_pid ──/control/setpoint──▶ pid_controller

pid_controller ──/control/effort──▶ sim_bridge_node
               ◀──/imu/data───────── sim_bridge_node

sim_bridge_node ──/cmd_vel──▶ simulator_node
                ◀──/sim2d/odom─────── simulator_node

simulator_node ──/sim2d/pose───▶ waypoint_navigator_node
               ──/sim2d/pose───▶ buoy_simulator_node
               ──/sim2d/odom───▶ sim_bridge_node
               ──/sim2d/path───▶ RViz2
               ──/sim2d/boat_shape─▶ RViz2
```

### Full topic reference

| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/nav/cmd_vel` | `Twist` | `waypoint_navigator` | `nav_to_pid` |
| `/control/setpoint` | `Twist` | `nav_to_pid` | `pid_controller` |
| `/control/effort` | `Twist` | `pid_controller` | `sim_bridge` |
| `/imu/data` | `Imu` | `sim_bridge` | `pid_controller` |
| `/cmd_vel` | `Twist` | `sim_bridge` | `simulator_node` |
| `/sim2d/pose` | `PoseStamped` | `simulator_node` | navigator, buoy sim |
| `/sim2d/odom` | `Odometry` | `simulator_node` | `sim_bridge` |
| `/sim2d/path` | `Path` | `simulator_node` | RViz2 |
| `/sim2d/navsat` | `NavSatFix` | `simulator_node` | — |
| `/sim2d/boat_shape` | `MarkerArray` | `simulator_node` | RViz2 |
| `/buoys/all` | `MarkerArray` | `buoy_simulator` | RViz2 |
| `/buoys/detected` | `MarkerArray` | `buoy_simulator` | RViz2 |
| `/gates/centers` | `Path` | `buoy_simulator` | `waypoint_navigator` |
| `/manual_mode` | `Bool` | `keyboard_teleop` | navigator, simulator |

---

## Tuning PID gains

The PID controller is in `src/asket_ec_sim2d/asket_ec_sim2d/pid_controller.py`:

```python
KP_SPEED, KI_SPEED, KD_SPEED = 0.5, 0.1, 0.0   # longitudinal (open loop)
KP_YAW,   KI_YAW,   KD_YAW   = 1.0, 0.0, 0.1   # yaw rate (closed loop via IMU)
```

The yaw loop is the important one — it closes on `angular_velocity.z` from `/imu/data` (synthesised from the simulator's odometry).

**Symptoms and fixes:**

| Symptom | Likely cause | Fix |
|---|---|---|
| Boat oscillates left/right | `KP_YAW` too high | Reduce `KP_YAW` |
| Boat turns slowly, overshoots gates | `KP_YAW` too low | Increase `KP_YAW` |
| Residual steady-state heading error | `KI_YAW` = 0 | Add small `KI_YAW` (e.g. 0.05) |
| Oscillation when correcting fast turns | `KD_YAW` too low | Increase `KD_YAW` |

After editing `pid_controller.py`, restart the launch — no rebuild needed.

---

## Manual keyboard control

While the simulation is running, open a separate terminal:

```bash
source /opt/ros/jazzy/setup.bash
~/.local/bin/keyboard_teleop_node
```

| Key | Action |
|---|---|
| Z / ↑ | Forward |
| S / ↓ | Backward |
| Q / ← | Turn left |
| D / → | Turn right |
| Space | Stop |
| M | Toggle MANUAL / AUTO |
| Ctrl+C | Quit |

In **AUTO** mode the Njord PID chain drives the boat. In **MANUAL** mode the keyboard takes over and the navigator pauses.

---

## Editing gates and waypoints

### Gate definitions — `config/buoys.yaml`

Each gate has one red buoy (starboard) and one green buoy (port). The boat navigates to the midpoint.

```yaml
gates:
  - id: 1
    red:   {lat: 41.3853, lon: 2.1733}
    green: {lat: 41.3853, lon: 2.1737}
```

`0.0001°` ≈ 11 metres. The current course has 3 gates spaced ~44 m apart north of the Barcelona harbour origin.

### Fallback waypoints — `config/waypoints.yaml`

Used only when `buoy_simulator_node` is not running.

---

## Target architecture — connecting to the real Njord stack

The eventual production setup runs the full Njord stack on a **Raspberry Pi 5** (or Jetson Orin) while this simulator runs on the laptop as a synthetic sensor provider.

```
┌─────────────────────────────────┐      LAN / same ROS_DOMAIN_ID
│  Laptop                         │◀────────────────────────────▶┐
│  asket_ec_sim2d (sim only)      │                              │
│  RViz2                          │                 ┌────────────┴──────────────┐
└─────────────────────────────────┘                 │  Raspberry Pi 5            │
                                                    │  ros2 launch bringup       │
                                                    │    njord.launch.py          │
                                                    │    use_sim_time:=true       │
                                                    │    enable_sensors:=false    │
                                                    │    enable_mavros:=false     │
                                                    └───────────────────────────┘
```

### Topics the Njord stack needs from this simulator

| Topic | Type | Status |
|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | ✅ `sim_bridge_node` |
| `/mavros/global_position/raw/fix` | `sensor_msgs/NavSatFix` | ⚠️ needs remap from `/sim2d/navsat` |
| `/scan` | `sensor_msgs/LaserScan` | ❌ not yet implemented |
| `/clock` | `rosgraph_msgs/Clock` | ❌ not yet implemented |

### Connecting RViz2 on the laptop to the Pi

Both machines must be on the same network and use the same domain ID:

```bash
# On both machines — must match
export ROS_DOMAIN_ID=42

# On the laptop — run only RViz2
source /opt/ros/jazzy/setup.bash
rviz2 -d ~/simulation_engineeringclub/asket_ec_sim_ws/src/asket_ec_sim2d/config/sim2d.rviz
```

No extra configuration needed — ROS2 DDS discovery is automatic on the same LAN.

---

## Known issues

| Issue | Impact | Fix |
|---|---|---|
| **Symlink workaround** required after every clean build | Extra manual step | Downgrade setuptools < 68 or migrate to `ament_cmake_python` |
| **`/mavros/global_position/raw/fix`** not published | Njord GPS nav won't work end-to-end | Add remap or relay node from `/sim2d/navsat` |
| **No `/scan` topic** | Nav2 costmap cannot see buoys as obstacles | Extend `buoy_simulator_node` to publish `LaserScan` |
| **No `/clock` topic** | `use_sim_time:=true` mode not supported | Add clock publisher to `simulator_node` |
| **No collision detection** | Boat can pass through buoys | Out of scope for 2D sim |

---

## Contributing

This project is maintained by members of **NODE Engineering Club**.
Open a pull request with a clear description of what changed and why.
If something is broken or confusing, open an issue.
