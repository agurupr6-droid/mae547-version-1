import importlib.resources as resources
import mujoco
import numpy as np

# ----- Same abstract configuration space as step 2 -----

JOINTS = {
    "lift": {
        "names": ["joint_lift"],
        "range": (0.0, 0.8),
    },
    "arm": {
        "names": ["joint_arm_l0", "joint_arm_l1", "joint_arm_l2", "joint_arm_l3"],
        "range": (0.0, 0.4),
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
    name_to_idx = {}
    for j in range(model.njnt):
        name = model.joint(j).name
        qpos_adr = model.jnt_qposadr[j]
        name_to_idx[name] = qpos_adr
    return name_to_idx


def clamp(x, lo, hi):
    return float(np.clip(x, lo, hi))


def q_to_qpos(model, data, name_to_idx, q):
    assert len(q) == len(Q_NAMES)
    q = dict(zip(Q_NAMES, q))

    qpos = data.qpos.copy()

    # lift
    lift = clamp(q["lift"], *JOINTS["lift"]["range"])
    for jname in JOINTS["lift"]["names"]:
        qpos[name_to_idx[jname]] = lift

    # arm (evenly distributed over 4 segments)
    arm_total = clamp(q["arm"], *JOINTS["arm"]["range"])
    arm_seg = arm_total / len(JOINTS["arm"]["names"])
    for jname in JOINTS["arm"]["names"]:
        qpos[name_to_idx[jname]] = arm_seg

    # head pan/tilt
    pan = clamp(q["head_pan"], *JOINTS["head_pan"]["range"])
    tilt = clamp(q["head_tilt"], *JOINTS["head_tilt"]["range"])
    for jname in JOINTS["head_pan"]["names"]:
        qpos[name_to_idx[jname]] = pan
    for jname in JOINTS["head_tilt"]["names"]:
        qpos[name_to_idx[jname]] = tilt

    # gripper (symmetrical opening)
    grip = clamp(q["grip"], *JOINTS["grip"]["range"])
    left, right = JOINTS["grip"]["names"]
    qpos[name_to_idx[left]] = grip
    qpos[name_to_idx[right]] = grip

    return qpos


def interpolate(q_start, q_goal, steps=50):
    q_start = np.asarray(q_start, dtype=float)
    q_goal = np.asarray(q_goal, dtype=float)
    alphas = np.linspace(0.0, 1.0, steps)
    return [(1 - a) * q_start + a * q_goal for a in alphas]


def is_collision(model, data, name_to_idx, q):
    # Convert planned joint configuration to full qpos
    qpos = q_to_qpos(model, data, name_to_idx, q)
    data.qpos[:] = qpos

    # Run physics
    mujoco.mj_forward(model, data)
    mujoco.mj_collision(model, data)

    # DEBUG: print every contact
    if data.ncon > 0:
        print(f"  q = {q}")
        print(f"  {data.ncon} contacts:")
        for c in range(data.ncon):
            contact = data.contact[c]
            name1 = model.geom(contact.geom1).name
            name2 = model.geom(contact.geom2).name
            print(f"     CONTACT: {name1} <-> {name2}")

    # Treat ANY non-floor contact as a "collision"
    for c in range(data.ncon):
        name1 = model.geom(data.contact[c].geom1).name
        name2 = model.geom(data.contact[c].geom2).name

        if "floor" in (name1 or "") or "floor" in (name2 or ""):
            continue  # ignore floor
        return True  # any other collision

    return False



def main():
    model, data = load_model()
    name_to_idx = get_qpos_indices(model)

    # Arm punch straight into the block
    q_start = np.array([0.20, 0.00, 0.0, 0.0, 0.0])   # lift, arm, head_pan, head_tilt, grip
    q_goal  = np.array([0.20, 0.40, 0.0, 0.0, 0.0])   # same lift, arm fully extended

    path = interpolate(q_start, q_goal, steps=30)
    print(f"Checking {len(path)} waypoints for collision...")

    colliding = []
    for i, q in enumerate(path):
        if is_collision(model, data, name_to_idx, q):
            colliding.append(i)

    if colliding:
        print(f"Waypoints in collision with obstacle: {colliding}")
        print(f"Total colliding waypoints: {len(colliding)}")
    else:
        print("No collisions detected along this path.")



if __name__ == "__main__":
    main()
