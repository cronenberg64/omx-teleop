# OpenMANIPULATOR-X Cartesian Teleoperation

Keyboard teleoperation for the **ROBOTIS OpenMANIPULATOR-X (OMX-X)** that controls the **end-effector in Cartesian space** (x, y, z + pitch + yaw + gripper). Inverse kinematics is solved with [ikpy](https://github.com/Phylliade/ikpy) directly from the robot's URDF — no MoveIt2 required.

---

## Dependencies

| Package | Purpose | Install |
|---|---|---|
| ROS2 (Humble / Iron / Jazzy) | Middleware | [ROS2 install guide](https://docs.ros.org/en/humble/Installation.html) |
| `open_manipulator` ROS2 packages | Controllers, joint-state publisher | cloned in this repo |
| `ikpy` | IK/FK solver | `pip install ikpy` (inside venv) |
| `numpy` | Math | ships with ikpy |

### Install dependencies

```bash
# Clone the repository
git clone https://github.com/ROBOTIS-GIT/open_manipulator.git

# Set up virtual environment and install python dependencies
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

---

## URDF

The script auto-discovers the URDF at:

```
omx-teleop/open_manipulator/open_manipulator_description/urdf/open_manipulator_x/open_manipulator_x.urdf
```

If you move the file, pass `--urdf /full/path/to/open_manipulator_x.urdf`.

---

## How to Launch

### Option A — direct Python (no ROS install step)

```bash
cd /path/to/omx-teleop
source .venv/bin/activate
source /opt/ros/<distro>/setup.bash

# With robot hardware already running bringup:
python3 omx_cartesian_teleop.py

# Override URDF path explicitly:
python3 omx_cartesian_teleop.py --urdf open_manipulator/open_manipulator_description/urdf/open_manipulator_x/open_manipulator_x.urdf
```

### Option B — ros2 run (after building the package)

```bash
# Copy the script into a ROS2 package (e.g., open_manipulator_teleop),
# add it to CMakeLists.txt / setup.py, then:
colcon build --packages-select open_manipulator_teleop
source install/setup.bash
ros2 run open_manipulator_teleop omx_cartesian_teleop
```

### Prerequisites (robot must already be up)

```bash
# Terminal 1 — hardware bringup
ros2 launch open_manipulator_bringup omx_bringup.launch.py

# Terminal 2 — this script
python3 omx_cartesian_teleop.py
```

---

## Keyboard Mapping

```
┌─────────────────────────────────────────────────────┐
│  W / S    End-effector forward / backward  (±X)     │
│  A / D    End-effector left   / right      (±Y)     │
│  Q / E    End-effector up     / down       (±Z)     │
│  R / F    Pitch up / down                           │
│  T / G    Yaw  left / right  (base rotation)        │
│  O        Gripper open                              │
│  P        Gripper close                             │
│  1        Step size 1 mm  / 0.5°                    │
│  2        Step size 5 mm  / 2.5°   (default)        │
│  3        Step size 10 mm / 5.0°                    │
│  ESC      Quit                                      │
└─────────────────────────────────────────────────────┘
```

---

## Architecture

```
Keyboard input
     │
     ▼
CartesianTeleop node
     │
     ├─ /joint_states subscriber
     │     └─ seeds target pose via FK on first message
     │
     ├─ IK solve (ikpy from URDF)
     │     ├─ convergence check  (FK error < 5 mm gate)
     │     ├─ joint-limit clamp  (URDF limits)
     │     └─ velocity clamp     (≤ 0.15 rad / step)
     │
     ├─ /arm_controller/joint_trajectory publisher
     └─ /gripper_controller/gripper_cmd action client
```

### IK approach

`ikpy` loads the URDF and auto-detects the kinematic chain from `link1` → `end_effector_link`. It treats all four revolute joints as active. The IK is seeded with the current joint configuration to avoid large jumps.

**Safety gates applied in order:**
1. **IK convergence check** — FK error of the solution must be < 5 mm; otherwise the command is dropped and a warning is printed.
2. **Joint-limit clamp** — each joint clamped to URDF limits before publishing.
3. **Velocity clamp** — each joint can move at most 0.15 rad per step (~8.6°).

---

## Known Limitations

### 1. The OMX-X is a 4-DOF arm

The OpenMANIPULATOR-X has **4 revolute joints** (joint1 = yaw, joint2–4 = pitch chain). It has **no dedicated wrist-roll joint**. This means:

- **Roll** cannot be controlled independently — it is always zero.
- The `T/G` yaw key moves joint1 (base); the `R/F` pitch key adjusts the wrist pitch via the IK solver. There is no separate wrist-yaw.
- Many Cartesian poses are **unreachable** — especially those requiring simultaneous x/y offset and a non-zero pitch.

### 2. IK is not guaranteed to converge

ikpy uses a numerical solver. Near singularities, the workspace boundary, or with conflicting position+orientation targets, IK may fail to converge. The script detects this and **will not move** the arm. Try smaller steps or a different orientation.

### 3. Orientation target is approximate

When you press `R/F/T/G`, the target rotation matrix is updated mathematically but the arm's actual orientation will only match if IK converges. After a failed IK, the target rotation is **not committed**, so the internal state stays consistent.

### 4. No collision checking

This script has no knowledge of obstacles or self-collision. Operate in a clear workspace and watch the arm at all times.

### 5. Gripper joint name varies by model

The gripper is addressed via the `GripperCommand` action (`/gripper_controller/gripper_cmd`), identical to the reference teleop. The `rh_r1_joint` feedback is used only to display the current gripper state; it is absent on some OMX-X configurations and is handled gracefully.

---

## File Structure

```
omx-teleop/
├── omx_cartesian_teleop.py       ← this script
├── README.md                     ← this file
└── open_manipulator/             ← cloned ROBOTIS repo
    └── open_manipulator_description/
        └── urdf/open_manipulator_x/
            └── open_manipulator_x.urdf   ← URDF used for IK
```