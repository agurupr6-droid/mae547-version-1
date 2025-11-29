#!/usr/bin/env python3
"""
step4_rrt_planner.py

Joint-space RRT planner for Stretch MuJoCo.

- Samples random valid joint configurations
- Grows a tree from start towards goal
- Uses your collision checker to ensure edges are free
- Saves the resulting path as rrt_path.npy

You will likely need to:
  * Hook in your real joint limits
  * Hook in your real collision checker
  * Plug this into your existing step1/2/3/5 files
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np


# ========= CONFIGURATION HOOKS (ADAPT TO YOUR PROJECT) =========

# Example joint limits: list of (min, max) for each joint
# TODO: Replace with your actual limits, or import them from step1_inspect_joints.py
JOINT_LIMITS: List[Tuple[float, float]] = [
    (-0.0, 1.1),   # lift
    (0.0, 0.5),    # arm
    (-1.5, 1.5),   # head_pan
    (-1.0, 0.5),   # head_tilt
    (-1.5, 1.5),   # wrist_yaw
]

STEP_SIZE = 0.1          # max distance for each tree extension (in joint space norm)
MAX_ITERS = 5000         # maximum number of RRT iterations
GOAL_SAMPLE_RATE = 0.1   # with this probability, directly sample the goal
GOAL_THRESHOLD = 0.1     # distance in joint space at which we consider we've reached the goal
EDGE_CHECK_RES = 10      # how many samples to check along each edge during collision checking


# ========= COLLISION CHECKING HOOK ==========

def is_state_in_collision(q: np.ndarray) -> bool:
    """
    Collision check hook.

    TODO: Replace this stub with your actual collision checker, e.g. a function
    from step3_collision_check.py that uses Stretch MuJoCo to test q.

    For now, this just assumes free space.
    """
    # Example if you already have something like:
    # from step3_collision_check import check_collision
    # return check_collision(q)
    return False


# ========= RRT CORE IMPLEMENTATION ==========

@dataclass
class Node:
    q: np.ndarray             # configuration (joint vector)
    parent: Optional[int]     # index into tree list, or None for root


def sample_random_config() -> np.ndarray:
    """Sample a random configuration within JOINT_LIMITS."""
    return np.array([
        random.uniform(jmin, jmax) for (jmin, jmax) in JOINT_LIMITS
    ], dtype=float)


def clip_to_limits(q: np.ndarray) -> np.ndarray:
    """Clamp configuration to joint limits."""
    q_clipped = np.empty_like(q)
    for i, (jmin, jmax) in enumerate(JOINT_LIMITS):
        q_clipped[i] = np.clip(q[i], jmin, jmax)
    return q_clipped


def distance(q1: np.ndarray, q2: np.ndarray) -> float:
    """Euclidean distance in joint space."""
    return float(np.linalg.norm(q1 - q2))


def steer(q_from: np.ndarray, q_to: np.ndarray, step_size: float) -> np.ndarray:
    """
    Move from q_from towards q_to by at most step_size (in joint-space distance).
    """
    d = distance(q_from, q_to)
    if d <= step_size:
        return clip_to_limits(q_to)
    direction = (q_to - q_from) / d
    q_new = q_from + direction * step_size
    return clip_to_limits(q_new)


def edge_is_collision_free(q1: np.ndarray, q2: np.ndarray) -> bool:
    """
    Check whether the straight-line edge between q1 and q2 is collision-free,
    by sampling intermediate configurations.
    """
    for i in range(EDGE_CHECK_RES + 1):
        alpha = i / EDGE_CHECK_RES
        q = (1 - alpha) * q1 + alpha * q2
        if is_state_in_collision(q):
            return False
    return True


def nearest_node_index(tree: List[Node], q: np.ndarray) -> int:
    """Return index of nearest node in tree to q."""
    dists = [distance(n.q, q) for n in tree]
    return int(np.argmin(dists))


def extract_path(tree: List[Node], goal_idx: int) -> np.ndarray:
    """
    Follow parents from goal_idx back to root, and return the path as
    an array of shape (N, dof).
    """
    indices = []
    idx = goal_idx
    while idx is not None:
        indices.append(idx)
        idx = tree[idx].parent
    indices.reverse()
    path = np.stack([tree[i].q for i in indices], axis=0)
    return path


def rrt_planner(
    q_start: Sequence[float],
    q_goal: Sequence[float],
    max_iters: int = MAX_ITERS,
    step_size: float = STEP_SIZE,
    goal_sample_rate: float = GOAL_SAMPLE_RATE,
    goal_threshold: float = GOAL_THRESHOLD,
) -> np.ndarray:
    """
    Basic joint-space RRT.

    Returns:
        path: np.ndarray of shape (N, dof) from q_start to q_goal.

    Raises:
        RuntimeError if no path found within max_iters.
    """
    q_start = clip_to_limits(np.array(q_start, dtype=float))
    q_goal = clip_to_limits(np.array(q_goal, dtype=float))

    if is_state_in_collision(q_start):
        raise RuntimeError("Start configuration is in collision.")
    if is_state_in_collision(q_goal):
        raise RuntimeError("Goal configuration is in collision.")

    tree: List[Node] = [Node(q=q_start, parent=None)]

    for it in range(max_iters):
        # Goal bias: sometimes sample the goal directly
        if random.random() < goal_sample_rate:
            q_rand = q_goal
        else:
            q_rand = sample_random_config()

        # Find nearest node and steer towards sample
        idx_near = nearest_node_index(tree, q_rand)
        q_near = tree[idx_near].q
        q_new = steer(q_near, q_rand, step_size)

        # Skip if new config collides or the edge intersects an obstacle
        if is_state_in_collision(q_new):
            continue
        if not edge_is_collision_free(q_near, q_new):
            continue

        # Add new node to tree
        tree.append(Node(q=q_new, parent=idx_near))

        # Check for reaching goal
        if distance(q_new, q_goal) < goal_threshold:
            # Optionally, try to directly connect to the exact goal
            if edge_is_collision_free(q_new, q_goal):
                tree.append(Node(q=q_goal, parent=len(tree) - 1))
                goal_idx = len(tree) - 1
            else:
                goal_idx = len(tree) - 1
            path = extract_path(tree, goal_idx)
            return path

    raise RuntimeError(f"RRT failed to find a path in {max_iters} iterations.")


# ========= SCRIPT ENTRY POINT ==========

def main():
    # TODO: Replace these with your real start/goal joint configs.
    # For example, you might read them from files produced in step1/2,
    # or from Stretch Mujoco directly.
    q_start = np.array([0.2, 0.0, 0.0, -0.2, 0.0])   # example start
    q_goal = np.array([0.8, 0.4, 0.5, -0.5, 1.0])    # example goal

    print("Planning RRT path...")
    path = rrt_planner(q_start, q_goal)
    print(f"Found path with {len(path)} waypoints.")

    # Save for step5 / run_sim to execute
    np.save("rrt_path.npy", path)
    print("Saved path to rrt_path.npy")


if __name__ == "__main__":
    main()
