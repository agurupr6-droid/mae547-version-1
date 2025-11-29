import importlib.resources as resources
import mujoco
import numpy as np

# ----- Configuration space definition -----

JOINTS = {
    "lift": {
        "names": ["joint_lift"],
        "range": (0.0, 0.8),
    },
    "arm": {
        # 4 telescoping links
        "names": ["joint_arm_l0", "joint_arm_l1", "joint_arm_l2", "joint_arm_l3"],
        "range": (0.0, 0.4),  # abstract total extension
    },
    "head_pan": {
        "names": ["joint_head_pan"],
        "range": (-1.5, 1.5),
    },
    "head_tilt": {
        "names": ["joint_head_tilt"],
        "range": (-0.9, 0.5),
    },
    "grip": {
        # control both fingers symmetrically
        "names": ["joint_gripper_finger_left_open", "joint_gripper_finger_right_open"],
        "range": (0.0, 0.5),
    },
}

Q_NAMES = ["lift", "arm", "head_pan", "head_tilt", "grip"]


def load_model():
    scene_xml = resources.files("stretch_mujoco") / "models" / "stretch.xml"
    print("Using XML:", scene_xml)
    model = mujoco.MjModel.from_xml_path(str(scene_xml))
    data = mujoco.MjData(model)
    return model, data


def get_qpos_indices(model):
    """Return a dict: joint_name -> qpos_index."""
    name_to_idx = {}
    for j in range(model.njnt):
        name = model.joint(j).name
        qpos_adr = model.jnt_qposadr[j]
        name_to_idx[name] = qpos_adr
    return name_to_idx


def clamp(x, lo, hi):
    return float(np.clip(x, lo, hi))


def q_to_qpos(model, data, name_to_idx, q):
    """
    Map abstract config q = [lift, arm, head_pan, head_tilt, grip]
    into MuJoCo's data.qpos.
    """
    assert len(q) == len(Q_NAMES)
    q = dict(zip(Q_NAMES, q))

    # Start from current qpos so we don't disturb base pose etc.
    qpos = data.qpos.copy()

    # Lift
    lift = clamp(q["lift"], *JOINTS["lift"]["range"])
    for jname in JOINTS["lift"]["names"]:
        qpos[name_to_idx[jname]] = lift

    # Arm: spread evenly across 4 links
    arm_total = clamp(q["arm"], *JOINTS["arm"]["range"])
    arm_seg = arm_total / len(JOINTS["arm"]["names"])
    for jname in JOINTS["arm"]["names"]:
        qpos[name_to_idx[jname]] = arm_seg

    # Head pan / tilt
    pan = clamp(q["head_pan"], *JOINTS["head_pan"]["range"])
    tilt = clamp(q["head_tilt"], *JOINTS["head_tilt"]["range"])
    for jname in JOINTS["head_pan"]["names"]:
        qpos[name_to_idx[jname]] = pan
    for jname in JOINTS["head_tilt"]["names"]:
        qpos[name_to_idx[jname]] = tilt

    # Gripper: symmetric open
    grip = clamp(q["grip"], *JOINTS["grip"]["range"])
    left_name, right_name = JOINTS["grip"]["names"]
    qpos[name_to_idx[left_name]] = grip
    qpos[name_to_idx[right_name]] = grip

    return qpos


def interpolate(q_start, q_goal, steps=50):
    q_start = np.asarray(q_start, dtype=float)
    q_goal = np.asarray(q_goal, dtype=float)
    alphas = np.linspace(0.0, 1.0, steps)
    return [(1 - a) * q_start + a * q_goal for a in alphas]


def main():
    model, data = load_model()
    name_to_idx = get_qpos_indices(model)

    # ----- Define start and goal configs in abstract joint space -----
    q_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0])           # near-home
    q_goal = np.array([0.4, 0.25, 0.8, -0.3, 0.4])          # reach forward

    print("q_start:", q_start)
    print("q_goal :", q_goal)

    path = interpolate(q_start, q_goal, steps=40)
    print(f"Generated {len(path)} waypoints in joint space.")

    # ----- Sanity check path in MuJoCo (no collisions yet) -----
    for i, q in enumerate(path):
        qpos = q_to_qpos(model, data, name_to_idx, q)
        data.qpos[:] = qpos
        mujoco.mj_forward(model, data)
        # later we'll add collision checking here

    print("Interpolation OK. (No collisions checked yet.)")


if __name__ == "__main__":
    main()
