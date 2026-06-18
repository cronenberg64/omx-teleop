#!/usr/bin/env python3
#
# Copyright 2024 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Jonathan Setiawan (Cartesian-space adaptation)
# Based on omx_f_teleop.py by Sungho Woo, Heewon Lee (ROBOTIS)
#
# Cartesian-space keyboard teleoperation for OpenMANIPULATOR-X (OMX-X).
# Uses ikpy for inverse kinematics from the OMX URDF.
#
# Keyboard mapping:
#   W / S  — end-effector forward / backward  (X axis)
#   A / D  — end-effector left   / right      (Y axis)
#   Q / E  — end-effector up     / down       (Z axis)
#   R / F  — pitch up / down                  (wrist pitch)
#   T / G  — yaw   left / right               (base rotation)
#   O / P  — gripper open / close
#   1      — step size 1 mm  / 0.5 deg
#   2      — step size 5 mm  / 2.5 deg
#   3      — step size 10 mm / 5.0 deg
#   ESC    — quit

import math
import os
import select
import sys
import termios
import threading
import time
import tty

import numpy as np

try:
    import ikpy.chain
    import ikpy.utils.math
except ImportError:
    sys.exit(
        "ERROR: ikpy not found.\n"
        "Install it inside your venv/conda env:\n"
        "  pip install ikpy"
    )

from control_msgs.action import GripperCommand
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ---------------------------------------------------------------------------
# OMX-X kinematic constants (from URDF)
# ---------------------------------------------------------------------------

# Absolute path to the static URDF shipped in the cloned repo.
# Adjust if you install the ROS package and want to use ament resource paths.
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_URDF_DEFAULT = os.path.join(
    _SCRIPT_DIR,
    "open_manipulator",
    "open_manipulator_description",
    "urdf",
    "open_manipulator_x",
    "open_manipulator_x.urdf",
)

# Ordered arm joint names as they appear in /joint_states
ARM_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]

# Per-joint limits [rad] — must match URDF
JOINT_LIMITS = [
    (-math.pi,  math.pi),   # joint1: yaw (z-axis)
    (-1.5,      1.5),        # joint2: pitch
    (-1.5,      1.4),        # joint3: pitch
    (-1.7,      1.97),       # joint4: pitch
]

# Maximum angular step per IK call [rad] — velocity safety clamp
MAX_JOINT_DELTA = 0.15  # ~8.6 deg per step (applied per individual joint)

# Gripper constants (matches reference teleop)
GRIPPER_JOINT_NAME = "rh_r1_joint"   # only present on OMX-F; ignored gracefully
GRIPPER_MAX = 1.1
GRIPPER_MIN = 0.0
GRIPPER_DELTA = 0.1

# Step-size presets [m] for translation, [rad] for rotation
STEP_PRESETS = {
    "1": (0.001, math.radians(0.5)),
    "2": (0.005, math.radians(2.5)),
    "3": (0.010, math.radians(5.0)),
}

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _build_ikpy_chain(urdf_path: str) -> ikpy.chain.Chain:
    """
    Load the kinematic chain from the OMX URDF.

    ikpy parses 6 links from the OMX-X URDF when base_elements=['link1']:
      index 0: 'Base link'           — fixed/passive  → inactive
      index 1: 'joint1'              — revolute        → ACTIVE
      index 2: 'joint2'              — revolute        → ACTIVE
      index 3: 'joint3'              — revolute        → ACTIVE
      index 4: 'joint4'              — revolute        → ACTIVE
      index 5: 'gripper_left_joint'  — prismatic       → inactive

    We must pass the mask explicitly; ikpy's auto-detection marks all
    links active by default which would include the fixed base and
    prismatic gripper joint.
    """
    import warnings
    active_mask = [False, True, True, True, True, False]
    with warnings.catch_warnings():
        # Suppress ikpy's spurious warning about the fixed base link
        warnings.simplefilter("ignore", UserWarning)
        chain = ikpy.chain.Chain.from_urdf_file(
            urdf_path,
            base_elements=["link1"],
            active_links_mask=active_mask,
        )
    return chain


def _fk(chain: ikpy.chain.Chain, joint_angles_4: list) -> tuple:
    """
    Forward kinematics: 4 joint angles → (position [m], rotation matrix 3x3).
    ikpy expects a angles vector of length = len(chain.links), with 0.0 for
    inactive (fixed / prismatic) links.
    """
    full = _to_ikpy_angles(chain, joint_angles_4)
    T = chain.forward_kinematics(full)          # 4×4 homogeneous matrix
    pos = T[:3, 3]
    rot = T[:3, :3]
    return pos, rot


def _rotation_to_rpy(R: np.ndarray) -> tuple:
    """ZYX Euler angles (roll, pitch, yaw) from a 3×3 rotation matrix."""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        roll  = math.atan2( R[2, 1],  R[2, 2])
        pitch = math.atan2(-R[2, 0],  sy)
        yaw   = math.atan2( R[1, 0],  R[0, 0])
    else:
        roll  = math.atan2(-R[1, 2],  R[1, 1])
        pitch = math.atan2(-R[2, 0],  sy)
        yaw   = 0.0
    return roll, pitch, yaw


def _to_ikpy_angles(chain: ikpy.chain.Chain, joint_angles_4: list) -> list:
    """
    Build the full-length angles vector that ikpy expects.
    Only revolute links (active_links_mask == True) get values from
    joint_angles_4; all others get 0.0.
    """
    full = [0.0] * len(chain.links)
    active_idx = [i for i, lk in enumerate(chain.links)
                  if chain.active_links_mask[i]]
    for k, idx in enumerate(active_idx):
        if k < len(joint_angles_4):
            full[idx] = joint_angles_4[k]
    return full


def _from_ikpy_angles(chain: ikpy.chain.Chain, full_angles: list) -> list:
    """Extract the 4 active joint values from an ikpy full-length angle list."""
    return [
        full_angles[i]
        for i, active in enumerate(chain.active_links_mask)
        if active
    ]


def _clamp_joints(angles: list) -> list:
    """Clamp each joint angle to its URDF limit."""
    return [
        float(np.clip(a, lo, hi))
        for a, (lo, hi) in zip(angles, JOINT_LIMITS)
    ]


def _velocity_clamp(current: list, target: list) -> list:
    """
    Limit how much any joint can move in a single step.
    Prevents the arm jumping to a far-away configuration.
    """
    result = []
    for c, t in zip(current, target):
        delta = t - c
        delta = max(-MAX_JOINT_DELTA, min(MAX_JOINT_DELTA, delta))
        result.append(c + delta)
    return result


def _clear_terminal():
    sys.stdout.write("\033[2J\033[H")
    sys.stdout.flush()


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

class CartesianTeleop(Node):

    def __init__(self, urdf_path: str):
        super().__init__("cartesian_teleop")

        # --- IK chain ---
        self.get_logger().info(f"Loading URDF from: {urdf_path}")
        try:
            self.chain = _build_ikpy_chain(urdf_path)
        except Exception as e:
            self.get_logger().fatal(f"Failed to load URDF: {e}")
            raise

        n_active = sum(self.chain.active_links_mask)
        self.get_logger().info(
            f"ikpy chain loaded. Links: {len(self.chain.links)}, "
            f"active (revolute) joints: {n_active}"
        )

        # --- ROS2 interfaces (same as reference) ---
        self.arm_publisher = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10
        )
        self.gripper_client = ActionClient(
            self, GripperCommand, "/gripper_controller/gripper_cmd"
        )
        self.subscription = self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )

        # --- State ---
        self.arm_joint_positions = [0.0] * 4
        self.gripper_position = 0.0
        self.joint_received = False

        # Current Cartesian target (updated from FK once joints are received)
        self.target_pos = np.zeros(3)           # [x, y, z] metres
        self.target_rot = np.eye(3)             # rotation matrix

        # Step sizes
        self.step_lin = 0.005   # 5 mm default
        self.step_ang = math.radians(2.5)
        self.step_label = "2 (5 mm / 2.5°)"

        # Rate limiting
        self.last_command_time = time.time()
        self.command_interval = 0.02            # 50 Hz max

        self.running = True

        self.get_logger().info("Waiting for /joint_states...")

    # ------------------------------------------------------------------
    # Subscriber callback
    # ------------------------------------------------------------------

    def _joint_state_callback(self, msg: JointState):
        # Update arm joint positions
        if set(ARM_JOINT_NAMES).issubset(set(msg.name)):
            for i, name in enumerate(ARM_JOINT_NAMES):
                idx = msg.name.index(name)
                self.arm_joint_positions[i] = msg.position[idx]

        # Update gripper (gracefully ignore if joint absent — OMX-X vs OMX-F)
        if GRIPPER_JOINT_NAME in msg.name:
            idx = msg.name.index(GRIPPER_JOINT_NAME)
            self.gripper_position = msg.position[idx]

        if not self.joint_received:
            # First message: seed the Cartesian target from real joint positions
            pos, rot = _fk(self.chain, self.arm_joint_positions)
            self.target_pos = pos.copy()
            self.target_rot = rot.copy()
            self.joint_received = True
            self.get_logger().info(
                f"Initial EE pose seeded from hardware: "
                f"pos={pos}, rpy={[math.degrees(v) for v in _rotation_to_rpy(rot)]}°"
            )

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------

    def _send_arm_command(self, joint_positions: list):
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = list(joint_positions)
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = 100_000_000  # 100 ms execution time
        msg.points.append(pt)
        self.arm_publisher.publish(msg)

    def _send_gripper_command(self):
        goal = GripperCommand.Goal()
        goal.command.position = self.gripper_position
        goal.command.max_effort = 10.0
        self.gripper_client.wait_for_server()
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

    # ------------------------------------------------------------------
    # IK solve + safety
    # ------------------------------------------------------------------

    def _solve_and_move(self, new_pos: np.ndarray, new_rot: np.ndarray) -> bool:
        """
        Run IK for (new_pos, new_rot). On success, clamp, velocity-limit,
        and publish. Returns True on success.
        """
        seed = _to_ikpy_angles(self.chain, self.arm_joint_positions)

        try:
            sol_full = self.chain.inverse_kinematics(
                target_position=new_pos,
                target_orientation=new_rot,
                orientation_mode="all",
                initial_position=seed,
            )
        except Exception as e:
            self._warn(f"IK exception: {e}")
            return False

        sol_4 = _from_ikpy_angles(self.chain, sol_full)

        # Check IK quality: FK error must be < 5 mm
        achieved_pos, _ = _fk(self.chain, sol_4)
        err = np.linalg.norm(achieved_pos - new_pos)
        if err > 0.005:
            self._warn(
                f"IK did not converge (position error {err*1000:.1f} mm > 5 mm). "
                f"Target may be out of workspace. NOT moving."
            )
            return False

        # Clamp to joint limits
        sol_clamped = _clamp_joints(sol_4)

        # Velocity clamp
        sol_safe = _velocity_clamp(self.arm_joint_positions, sol_clamped)

        # Commit and send
        self.arm_joint_positions = sol_safe
        self.target_pos = new_pos.copy()
        self.target_rot = new_rot.copy()
        self._send_arm_command(sol_safe)
        return True

    # ------------------------------------------------------------------
    # Terminal display
    # ------------------------------------------------------------------

    def _display(self):
        pos, rot = _fk(self.chain, self.arm_joint_positions)
        roll, pitch, yaw = _rotation_to_rpy(rot)

        _clear_terminal()
        print("=" * 60)
        print("  OpenMANIPULATOR-X  |  Cartesian Teleoperation")
        print("=" * 60)
        print(f"  Step size : {self.step_label}")
        print()
        print("  End-Effector Pose (FK from current joints):")
        print(f"    X      : {pos[0]*1000:+8.2f} mm")
        print(f"    Y      : {pos[1]*1000:+8.2f} mm")
        print(f"    Z      : {pos[2]*1000:+8.2f} mm")
        print(f"    Roll   : {math.degrees(roll):+8.2f}°")
        print(f"    Pitch  : {math.degrees(pitch):+8.2f}°")
        print(f"    Yaw    : {math.degrees(yaw):+8.2f}°")
        print()
        print("  Joint angles [deg]:")
        for i, (name, angle) in enumerate(
            zip(ARM_JOINT_NAMES, self.arm_joint_positions)
        ):
            lo, hi = JOINT_LIMITS[i]
            print(
                f"    {name}: {math.degrees(angle):+7.2f}°  "
                f"[{math.degrees(lo):.0f}° .. {math.degrees(hi):.0f}°]"
            )
        print(f"    gripper : {self.gripper_position:+6.3f}")
        print()
        print("  Keys: W/S=X  A/D=Y  Q/E=Z  R/F=Pitch  T/G=Yaw  O/P=Gripper")
        print("        1/2/3 = step size   ESC = quit")
        print("=" * 60)

    def _warn(self, msg: str):
        """Print a warning below the status display."""
        print(f"\n  ⚠  {msg}")
        sys.stdout.flush()

    # ------------------------------------------------------------------
    # Keyboard input
    # ------------------------------------------------------------------

    @staticmethod
    def _get_key(timeout=0.05) -> str | None:
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            if rlist:
                return sys.stdin.read(1)
            return None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self):
        # Wait for first /joint_states
        while not self.joint_received and rclpy.ok() and self.running:
            self.get_logger().info("Waiting for initial joint states...")
            rclpy.spin_once(self, timeout_sec=1.0)

        if not rclpy.ok() or not self.running:
            return

        self.get_logger().info("Joints received — entering Cartesian teleop loop.")
        self._display()

        try:
            while rclpy.ok() and self.running:
                key = self._get_key()
                now = time.time()

                if key is None:
                    # Keep ROS spinning so callbacks stay alive
                    rclpy.spin_once(self, timeout_sec=0.0)
                    continue

                if now - self.last_command_time < self.command_interval:
                    continue

                # ---- Step size toggle ----
                if key in STEP_PRESETS:
                    self.step_lin, self.step_ang = STEP_PRESETS[key]
                    labels = {"1": "1 (1 mm / 0.5°)",
                              "2": "2 (5 mm / 2.5°)",
                              "3": "3 (10 mm / 5.0°)"}
                    self.step_label = labels[key]
                    self._display()
                    continue

                # ---- ESC ----
                if key == "\x1b":
                    self.running = False
                    break

                # ---- Compute new target ----
                new_pos = self.target_pos.copy()
                new_rot = self.target_rot.copy()

                moved = False

                # Translation: W/S = ±X, A/D = ±Y, Q/E = ±Z
                if   key == "w": new_pos[0] += self.step_lin;  moved = True
                elif key == "s": new_pos[0] -= self.step_lin;  moved = True
                elif key == "a": new_pos[1] += self.step_lin;  moved = True
                elif key == "d": new_pos[1] -= self.step_lin;  moved = True
                elif key == "q": new_pos[2] += self.step_lin;  moved = True
                elif key == "e": new_pos[2] -= self.step_lin;  moved = True

                # Rotation: R/F = pitch, T/G = yaw (about world Z via joint1)
                elif key in ("r", "f", "t", "g"):
                    dang = self.step_ang
                    if key == "r":
                        # Pitch up: rotate target_rot about its own Y axis
                        new_rot = new_rot @ _Ry( dang)
                    elif key == "f":
                        new_rot = new_rot @ _Ry(-dang)
                    elif key == "t":
                        # Yaw left: rotate about world Z
                        new_rot = _Rz( dang) @ new_rot
                    elif key == "g":
                        new_rot = _Rz(-dang) @ new_rot
                    moved = True

                # Gripper
                elif key == "o":
                    self.gripper_position = min(
                        self.gripper_position + GRIPPER_DELTA, GRIPPER_MAX
                    )
                    self._send_gripper_command()
                    self._display()
                    self.last_command_time = now
                    continue
                elif key == "p":
                    self.gripper_position = max(
                        self.gripper_position - GRIPPER_DELTA, GRIPPER_MIN
                    )
                    self._send_gripper_command()
                    self._display()
                    self.last_command_time = now
                    continue

                if moved:
                    ok = self._solve_and_move(new_pos, new_rot)
                    if not ok:
                        time.sleep(0.5)  # let the warning be readable
                    self._display()
                    self.last_command_time = now

        except Exception as exc:
            self.get_logger().error(f"Exception in run loop: {exc}")
            raise


# ---------------------------------------------------------------------------
# Rotation helpers
# ---------------------------------------------------------------------------

def _Rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)

def _Ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)

def _Rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="Cartesian-space keyboard teleop for OpenMANIPULATOR-X"
    )
    parser.add_argument(
        "--urdf",
        default=_URDF_DEFAULT,
        help=(
            "Path to open_manipulator_x.urdf "
            f"(default: {_URDF_DEFAULT})"
        ),
    )
    # Filter out ROS2 args before parsing
    args, _ = parser.parse_known_args()

    if not os.path.isfile(args.urdf):
        sys.exit(
            f"ERROR: URDF not found at:\n  {args.urdf}\n"
            "Pass --urdf /path/to/open_manipulator_x.urdf"
        )

    rclpy.init()
    node = CartesianTeleop(urdf_path=args.urdf)

    thread = threading.Thread(target=node.run, daemon=True)
    thread.start()

    try:
        while thread.is_alive():
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nCtrl+C — shutting down...")
        node.running = False
        thread.join(timeout=3.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
