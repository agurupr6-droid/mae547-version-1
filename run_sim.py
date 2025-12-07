from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass
from typing import List, Sequence, Tuple

from stretch_mujoco import StretchMujocoSimulator
from stretch_mujoco.enums.actuators import Actuators

# ---------------------------------------------------------------------------
#  ENV + RRT CONFIGURATION
# ---------------------------------------------------------------------------

# Object pose (center of the blue block from scene.xml)
# Adjusted Y slightly to account for gripper alignment
OBJECT_POSITION = (1.20, 0.10)
OBJECT_HEIGHT = 0.40

# Desired standoff distance so the arm can reach straight forward to the block
ARM_REACH_TARGET = 0.58  # meters of effective arm extension to the block centerline
BASE_GOAL_POSITION = (OBJECT_POSITION[0] - ARM_REACH_TARGET, OBJECT_POSITION[1] - 0.05)

# Offset between the base center and the arm column mount (from stretch.xml @ link_lift).
# The lateral component matters for aligning the gripper with the block without scraping.
# Increased to ensure gripper clears the table edge when approaching the block.
ARM_MOUNT_LATERAL_OFFSET = 0.60

# Bounds that enclose the useful driving workspace (meters)
WORKSPACE_BOUNDS = (-0.5, 1.8, -1.0, 1.0)  # xmin, xmax, ymin, ymax

# Collision margin around the base footprint
ROBOT_RADIUS = 0.23

# Obstacles present in the scene (red cylinder, etc.)
CIRCLE_OBSTACLES = [
    {"center": (0.35, 0.25), "radius": 0.10 + ROBOT_RADIUS},  # red cylinder
    {"center": OBJECT_POSITION, "radius": 0.25},  # keep base off the table surface
]
OBSTACLE_STOP_MARGIN = -0.05  # shrink runtime detection radius to avoid false positives

random.seed(3)


@dataclass
class RRTParams:
    step_size: float = 0.25
    max_iters: int = 6000
    goal_sample_rate: float = 0.15
    goal_threshold: float = 0.20


@dataclass
class RRTNode:
    point: Tuple[float, float]
    parent: int | None


# ---------------------------------------------------------------------------
#  RRT UTILITIES
# ---------------------------------------------------------------------------

def sample_point() -> Tuple[float, float]:
    xmin, xmax, ymin, ymax = WORKSPACE_BOUNDS
    return random.uniform(xmin, xmax), random.uniform(ymin, ymax)


def distance(p1: Sequence[float], p2: Sequence[float]) -> float:
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def steer(p_from: Sequence[float], p_to: Sequence[float], step: float) -> Tuple[float, float]:
    dist = distance(p_from, p_to)
    if dist <= step:
        return float(p_to[0]), float(p_to[1])
    ratio = step / dist
    x = p_from[0] + (p_to[0] - p_from[0]) * ratio
    y = p_from[1] + (p_to[1] - p_from[1]) * ratio
    return x, y


def point_in_collision(point: Sequence[float]) -> bool:
    x, y = point
    xmin, xmax, ymin, ymax = WORKSPACE_BOUNDS
    if x < xmin + ROBOT_RADIUS or x > xmax - ROBOT_RADIUS:
        return True
    if y < ymin + ROBOT_RADIUS or y > ymax - ROBOT_RADIUS:
        return True

    for obs in CIRCLE_OBSTACLES:
        cx, cy = obs["center"]
        if math.hypot(x - cx, y - cy) <= obs["radius"]:
            return True

    return False


def detect_nearby_obstacle(sim: StretchMujocoSimulator) -> dict | None:
    """Check live base pose against obstacle list; return obstacle dict if too close."""
    x, y, _ = sim.get_base_pose()
    for obs in CIRCLE_OBSTACLES:
        cx, cy = obs["center"]
        if math.hypot(x - cx, y - cy) <= obs["radius"] + OBSTACLE_STOP_MARGIN:
            return obs
    return None


def edge_in_collision(p1: Sequence[float], p2: Sequence[float], step: float = 0.05) -> bool:
    dist = max(distance(p1, p2), 1e-6)
    samples = int(dist / step) + 1
    for i in range(samples + 1):
        alpha = i / samples
        x = p1[0] + (p2[0] - p1[0]) * alpha
        y = p1[1] + (p2[1] - p1[1]) * alpha
        if point_in_collision((x, y)):
            return True
    return False


def nearest_idx(nodes: List[RRTNode], point: Sequence[float]) -> int:
    dists = [distance(n.point, point) for n in nodes]
    return int(min(range(len(dists)), key=dists.__getitem__))


def extract_path(nodes: List[RRTNode], goal_idx: int) -> List[Tuple[float, float]]:
    ordered: List[Tuple[float, float]] = []
    idx: int | None = goal_idx
    while idx is not None:
        ordered.append(nodes[idx].point)
        idx = nodes[idx].parent
    ordered.reverse()
    return ordered


def shortcut_path(path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    if len(path) <= 2:
        return path
    simplified = [path[0]]
    idx = 0
    while idx < len(path) - 1:
        next_idx = len(path) - 1
        while next_idx > idx + 1 and edge_in_collision(path[idx], path[next_idx]):
            next_idx -= 1
        simplified.append(path[next_idx])
        idx = next_idx
    cleaned = [simplified[0]]
    for pt in simplified[1:]:
        if distance(pt, cleaned[-1]) > 1e-3:
            cleaned.append(pt)
    return cleaned


def plan_rrt_path(start: Tuple[float, float], goal: Tuple[float, float], params: RRTParams) -> List[Tuple[float, float]]:
    if point_in_collision(start):
        raise RuntimeError("Start pose is in collision; cannot plan.")
    if point_in_collision(goal):
        raise RuntimeError("Goal pose is in collision; cannot plan.")

    tree: List[RRTNode] = [RRTNode(point=start, parent=None)]

    for _ in range(params.max_iters):
        if random.random() < params.goal_sample_rate:
            sample = goal
        else:
            sample = sample_point()

        idx = nearest_idx(tree, sample)
        candidate = steer(tree[idx].point, sample, params.step_size)

        if point_in_collision(candidate):
            continue
        if edge_in_collision(tree[idx].point, candidate):
            continue

        tree.append(RRTNode(point=candidate, parent=idx))

        if distance(candidate, goal) < params.goal_threshold and not edge_in_collision(candidate, goal):
            tree.append(RRTNode(point=goal, parent=len(tree) - 1))
            raw_path = extract_path(tree, len(tree) - 1)
            return shortcut_path(raw_path)

    raise RuntimeError("RRT failed to find a collision-free base path.")


# ---------------------------------------------------------------------------
#  BASE EXECUTION HELPERS
# ---------------------------------------------------------------------------

def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def heading_with_lateral_offset(
    base_xy: Tuple[float, float], target_xy: Tuple[float, float], lateral_offset: float
) -> float:
    """
    Compute heading so the base's forward axis (and thus the arm) points to the target,
    compensating for the arm column sitting off-center on the chassis.
    """
    dx = target_xy[0] - base_xy[0]
    dy = target_xy[1] - base_xy[1]
    if abs(lateral_offset) < 1e-6:
        return math.atan2(dy, dx)

    dist_sq = dx * dx + dy * dy
    # If we're essentially on top of the target, fall back to the naive heading.
    if dist_sq <= lateral_offset * lateral_offset + 1e-8:
        return math.atan2(dy, dx)

    forward_component = math.sqrt(max(dist_sq - lateral_offset * lateral_offset, 1e-8))
    return wrap_angle(math.atan2(dy, dx) + math.atan2(lateral_offset, forward_component))


def move_base_to(
    sim: StretchMujocoSimulator,
    target_xy: Tuple[float, float],
    linear_speed: float = 0.6,
    angular_speed: float = 1.2,
) -> bool:
    pos_tol = 0.03
    heading_tol = 0.02
    while sim.is_running():
        x, y, theta = sim.get_base_pose()
        dx = target_xy[0] - x
        dy = target_xy[1] - y
        dist = math.hypot(dx, dy)

        obstacle = detect_nearby_obstacle(sim)
        if obstacle:
            print(f"Obstacle detected near {obstacle['center']} — pausing to replan.")
            sim.set_base_velocity(0.0, 0.0)
            return False

        if dist <= pos_tol:
            break

        desired_heading = math.atan2(dy, dx)
        heading_error = wrap_angle(desired_heading - theta)

        if abs(heading_error) > heading_tol:
            omega = angular_speed if heading_error > 0 else -angular_speed
            sim.set_base_velocity(0.0, omega)
        else:
            sim.set_base_velocity(linear_speed, 0.0)

        time.sleep(0.02)

    sim.set_base_velocity(0.0, 0.0)
    return True


def execute_base_path(
    sim: StretchMujocoSimulator,
    path: List[Tuple[float, float]],
    final_goal: Tuple[float, float],
    max_replans: int = 3,
) -> bool:
    if len(path) <= 1:
        return True

    replans = 0
    idx = 1

    while idx < len(path):
        waypoint = path[idx]
        print(f"  -> Waypoint {idx}/{len(path) - 1}: {waypoint}")
        reached = move_base_to(sim, waypoint)
        if reached:
            idx += 1
            continue

        if replans >= max_replans:
            print("Maximum replans reached; aborting this path.")
            sim.set_base_velocity(0.0, 0.0)
            return False

        replans += 1
        current_xy = sim.get_base_pose()[:2]
        print(f"    Replanning path ({replans}/{max_replans}) from {current_xy} to {final_goal}...")
        try:
            path = plan_rrt_path(current_xy, final_goal, RRTParams())
            print(f"    New path has {len(path)} waypoints.")
            idx = 1
        except RuntimeError as err:
            print(f"    Replan failed: {err}")
            sim.set_base_velocity(0.0, 0.0)
            return False

    sim.set_base_velocity(0.0, 0.0)
    return True


def run_manipulation_sequence(sim: StretchMujocoSimulator) -> None:
    """Drive the arm to grasp the blue block and hold it securely."""
    lift_approach = OBJECT_HEIGHT - 0.12
    lift_pick = OBJECT_HEIGHT - 0.16
    lift_carry = 0.70
    arm_extend = ARM_REACH_TARGET - 0.30  # Position gripper to close around block
    arm_carry = 0.34

    print("Opening gripper...")
    sim.move_to(Actuators.gripper, 0.09)  # Open wider to grab the block
    sim.wait_until_at_setpoint(Actuators.gripper)

    print("Lowering wrist and lift to picking height...")
    sim.move_to(Actuators.wrist_yaw, 0.0)
    sim.move_to(Actuators.lift, lift_approach)
    sim.wait_until_at_setpoint(Actuators.lift)

    print("Extending arm toward the block slowly...")
    # Extend in smaller increments to avoid pushing the block
    current_arm = 0.0
    while current_arm < arm_extend:
        current_arm = min(current_arm + 0.05, arm_extend)
        sim.move_to(Actuators.arm, current_arm)
        time.sleep(0.15)
    sim.wait_until_at_setpoint(Actuators.arm)

    print("Dropping lift slightly to wrap around the block...")
    sim.move_to(Actuators.lift, lift_pick)
    sim.wait_until_at_setpoint(Actuators.lift)
    time.sleep(0.1)

    print("Closing gripper to grasp object...")
    sim.move_to(Actuators.gripper, 0.0)
    sim.wait_until_at_setpoint(Actuators.gripper)
    time.sleep(0.2)

    print("Lifting object and retracting for travel...")
    sim.move_to(Actuators.lift, lift_carry)
    sim.move_to(Actuators.arm, arm_carry)
    sim.wait_until_at_setpoint(Actuators.lift)
    sim.wait_until_at_setpoint(Actuators.arm)
    time.sleep(0.2)


def align_base_heading(sim: StretchMujocoSimulator, target_theta: float, angular_speed: float = 1.0) -> None:
    """Rotate the base back to its desired heading."""
    heading_tol = 0.015
    while sim.is_running():
        _, _, theta = sim.get_base_pose()
        error = wrap_angle(target_theta - theta)
        if abs(error) < heading_tol:
            break
        omega = angular_speed if error > 0 else -angular_speed
        sim.set_base_velocity(0.0, omega)
        time.sleep(0.02)
    sim.set_base_velocity(0.0, 0.0)


def face_point(
    sim: StretchMujocoSimulator,
    target_xy: Tuple[float, float],
    angular_speed: float = 1.0,
    compensate_arm_offset: bool = False,
) -> None:
    """Rotate the base so it faces the provided XY point."""
    x, y, _ = sim.get_base_pose()
    if compensate_arm_offset:
        desired_heading = heading_with_lateral_offset(
            (x, y), target_xy, ARM_MOUNT_LATERAL_OFFSET
        )
    else:
        desired_heading = math.atan2(target_xy[1] - y, target_xy[0] - x)
    align_base_heading(sim, desired_heading, angular_speed)


# ---------------------------------------------------------------------------
#  MAIN SIMULATION LOGIC
# ---------------------------------------------------------------------------

def main() -> None:
    sim = StretchMujocoSimulator()

    print("Starting simulator...")
    sim.start(headless=False)

    if not sim.is_running():
        print("Failed to start simulator.")
        return

    sim.home()
    time.sleep(1.0)

    start_pose = sim.get_base_pose()
    start_xy = start_pose[:2]
    start_heading = start_pose[2]
    goal_xy = BASE_GOAL_POSITION

    print("Planning RRT base path...")
    try:
        path = plan_rrt_path(start_xy, goal_xy, RRTParams())
    except RuntimeError as err:
        print(f"RRT planning failed: {err}")
        sim.stop()
        return

    print(f"Planned path with {len(path)} waypoints.")
    reached_goal = execute_base_path(sim, path, goal_xy)

    if not reached_goal:
        print("Could not reach goal because an obstacle was detected. Press ENTER to exit.")
        input()
        sim.stop()
        return

    face_point(sim, OBJECT_POSITION, compensate_arm_offset=True)
    run_manipulation_sequence(sim)

    print("Planning return path to start pose...")
    try:
        return_path = plan_rrt_path(sim.get_base_pose()[:2], start_xy, RRTParams())
    except RuntimeError as err:
        print(f"Return path planning failed: {err}")
        print("Robot will stay near the object. Press ENTER to exit.")
        input()
        sim.stop()
        return

    returned_home = execute_base_path(sim, return_path, start_xy)
    align_base_heading(sim, start_heading)

    if returned_home:
        print("Back at start pose with the block in hand. Press ENTER to exit.")
    else:
        print("Return path interrupted by obstacle detection. Press ENTER to exit.")
    input()
    sim.stop()


if __name__ == "__main__":
    main()
