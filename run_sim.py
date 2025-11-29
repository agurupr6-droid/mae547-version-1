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

# Goal is the blue box defined in models/scene.xml
GOAL_POSITION = (1.20, 0.00)

# Bounds that enclose the useful driving workspace (meters)
WORKSPACE_BOUNDS = (-0.5, 1.8, -1.0, 1.0)  # xmin, xmax, ymin, ymax

# Collision margin around the base footprint
ROBOT_RADIUS = 0.23

# Obstacles present in the scene (red cylinder, etc.)
CIRCLE_OBSTACLES = [
    {"center": (0.35, 0.25), "radius": 0.10 + ROBOT_RADIUS},  # radius from XML + margin
]
OBSTACLE_STOP_MARGIN = 0.02  # extra detection layer

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


def move_base_to(
    sim: StretchMujocoSimulator,
    target_xy: Tuple[float, float],
    linear_speed: float = 0.6,
    angular_speed: float = 1.2,
) -> None:
    pos_tol = 0.03
    heading_tol = 0.02
    while sim.is_running():
        x, y, theta = sim.get_base_pose()
        dx = target_xy[0] - x
        dy = target_xy[1] - y
        dist = math.hypot(dx, dy)

        obstacle = detect_nearby_obstacle(sim)
        if obstacle:
            print(f"Obstacle detected near {obstacle['center']} — stopping base.")
            break

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


def execute_base_path(sim: StretchMujocoSimulator, path: List[Tuple[float, float]]) -> None:
    if len(path) <= 1:
        return
    print(f"Executing {len(path)} waypoints...")
    for idx, waypoint in enumerate(path[1:], start=1):
        print(f"  -> Waypoint {idx}/{len(path) - 1}: {waypoint}")
        move_base_to(sim, waypoint)
        if detect_nearby_obstacle(sim):
            print("Navigation halted because an obstacle is too close.")
            break
    sim.set_base_velocity(0.0, 0.0)


def run_manipulation_sequence(sim: StretchMujocoSimulator) -> None:
    """Simple arm motion to show the robot can work at the goal."""
    print("Preparing manipulator for pickup demo...")
    sim.move_to(Actuators.gripper, 0.04)
    sim.wait_until_at_setpoint(Actuators.gripper)

    sim.move_to(Actuators.wrist_yaw, 0.0)
    sim.move_to(Actuators.lift, 0.55)
    sim.move_to(Actuators.arm, 0.45)

    sim.wait_until_at_setpoint(Actuators.lift)
    sim.wait_until_at_setpoint(Actuators.arm)
    time.sleep(0.4)


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

    start_xy = sim.get_base_pose()[:2]
    goal_xy = GOAL_POSITION

    print("Planning RRT base path...")
    try:
        path = plan_rrt_path(start_xy, goal_xy, RRTParams())
    except RuntimeError as err:
        print(f"RRT planning failed: {err}")
        sim.stop()
        return

    print(f"Planned path with {len(path)} waypoints.")
    execute_base_path(sim, path)

    run_manipulation_sequence(sim)

    if detect_nearby_obstacle(sim):
        print("Robot stopped early due to obstacle detection. Press ENTER to exit.")
    else:
        print("Robot is at goal. Press ENTER to exit.")
    input()
    sim.stop()


if __name__ == "__main__":
    main()
