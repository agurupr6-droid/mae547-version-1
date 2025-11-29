import time
import importlib.resources as resources
from stretch_mujoco import StretchMujocoSimulator

from iris_prm import build_prm_and_plan, pixel_to_world  # you’ll define pixel_to_world there

def main():
    # --- start sim ---
    scene_xml = resources.files("stretch_mujoco") / "models" / "stretch.xml"
    sim = StretchMujocoSimulator(scene_xml_path=str(scene_xml))
    sim.start(headless=False)
    sim.home()
    time.sleep(1)

    # --- choose start & goal in IMAGE PIXELS (must be free points) ---
    start_xy = (503.97, 224.43)      # roughly where robot starts in the map
    goal_xy  = (1265.91, 921.73)     # where you want it to end

    path, obstacle_map = build_prm_and_plan(start_xy, goal_xy)
    if not path:
        print("No PRM path found")
        return

    # convert all PRM nodes to world coordinates (meters)
    waypoints = [pixel_to_world(n, obstacle_map) for n in path]

    # --- simple path-following controller ---
    # internal belief of robot pose (start at 0,0, facing +x)
    x, y, yaw = 0.0, 0.0, 0.0

    for (x_goal, y_goal) in waypoints:
        dx = x_goal - x
        dy = y_goal - y
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            continue

        desired_yaw = math.atan2(dy, dx)
        d_yaw = desired_yaw - yaw

        # 1) rotate to face waypoint
        #   (you may need to change "base_rotate" to whatever your API uses)
        sim.move_by("base_rotate", d_yaw)
        sim.wait_while_is_moving("base_rotate")

        # 2) drive straight toward waypoint
        sim.move_by("base_translate", dist)
        sim.wait_while_is_moving("base_translate")

        # update our internal pose estimate
        yaw = desired_yaw
        x, y = x_goal, y_goal

    print("Finished PRM path. Press Ctrl+C to exit.")
    try:
        while sim.is_running():
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    sim.stop()

if __name__ == "__main__":
    main()

