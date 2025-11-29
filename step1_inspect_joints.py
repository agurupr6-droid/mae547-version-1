import importlib.resources as resources
import mujoco
import numpy as np

# Load the same robot-only XML you use in run_sim.py
scene_xml = resources.files("stretch_mujoco") / "models" / "stretch.xml"
print("Using XML:", scene_xml)

model = mujoco.MjModel.from_xml_path(str(scene_xml))
data = mujoco.MjData(model)

print("\nJoints:")
for j in range(model.njnt):
    name = model.joint(j).name
    qpos_adr = model.jnt_qposadr[j]
    qpos_dim = model.jnt_dofadr[j] - model.jnt_qposadr[j] + 1 if j < model.njnt - 1 else model.nq - model.jnt_qposadr[j]
    # Limits (if defined)
    range_lo, range_hi = model.jnt_range[j]
    print(f"{j:2d}  name={name:15s} qpos_index={qpos_adr}  range=({range_lo:.3f}, {range_hi:.3f})")
