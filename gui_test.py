import mujoco
import mujoco.viewer

print("Loading minimal MuJoCo model...")

xml = """
<mujoco>
  <worldbody>
    <geom type="box" size="0.1 0.1 0.1" rgba="0 1 0 1"/>
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

print("Launching viewer...")
mujoco.viewer.launch(model, data)
