import time
import importlib.resources as resources
import numpy as np

from stretch_mujoco import StretchMujocoSimulator

# Same order we used for q: [lift, arm, head_pan, head_tilt, grip]
Q_NAMES = ["lift", "arm", "head_pan", "head_tilt", "grip"]


def load_scene_xml():
    # Use the same robot-only model with your obstacle
    return resources.files("stretch_mujoco") / "models" / "stretch.xml"


def main():
    # 1. Load planned path
    path = np.load("rrt_path.npy")
    print(f"Loaded RRT path with shape {path.shape}")

    # 2. Start simulator
    scene_xml = load_scene_xml()
    print("Using XML:", scene_xml)

    sim = StretchMujocoSimulator(scene_xml_path=str(scene_xml))
    sim.start(headless=False)

    print("Homing robot...")
    sim.home()
    time.sleep(1.0)

       # interpolation for smooth execution
    def interpolate_segment(q_from, q_to, steps=50):
        alphas = np.linspace(0.0, 1.0, steps)
        return [(1 - a) * q_from + a * q_to for a in alphas]

    print("Executing RRT path (smooth interpolation)...")

    for seg_idx in range(len(path) - 1):
        q_from = path[seg_idx]
        q_to   = path[seg_idx + 1]

        segment = interpolate_segment(q_from, q_to, steps=50)

        for q in segment:
            lift, arm, head_pan, head_tilt, grip = q

            sim.move_to("lift",     float(lift))
            sim.move_to("arm",      float(arm))
            sim.move_to("head_pan", float(head_pan))
            sim.move_to("head_tilt", float(head_tilt))
            sim.move_to("gripper",  float(grip))

            time.sleep(0.02)  # <<< smooth motion

        print(f"Segment {seg_idx+1}/{len(path)-1} done.")


    print("Path execution done. Viewer will stay open; press Ctrl+C in the terminal to quit.")

    try:
        while sim.is_running():
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Stopping sim...")

    sim.stop()
    print("Done.")


if __name__ == "__main__":
    main()
