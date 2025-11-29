from stretch_mujoco import StretchMujocoSimulator
from stretch_mujoco.enums.actuators import Actuators
import time

# ---------------------------------------------------------
#  BASE MOVEMENT USING set_base_velocity
# ---------------------------------------------------------

def drive_forward(sim, speed=0.7, duration=1.0):
    """Drive straight forward in robot +x."""
    sim.set_base_velocity(speed, 0.0)
    start = time.time()
    while time.time() - start < duration and sim.is_running():
        time.sleep(0.01)
    sim.set_base_velocity(0.0, 0.0)


def turn_left(sim, speed=1.0, duration=1.0):
    """Rotate CCW (left)."""
    sim.set_base_velocity(0.0, speed)
    start = time.time()
    while time.time() - start < duration and sim.is_running():
        time.sleep(0.01)
    sim.set_base_velocity(0.0, 0.0)


def turn_right(sim, speed=1.0, duration=1.0):
    """Rotate CW (right)."""
    sim.set_base_velocity(0.0, -speed)
    start = time.time()
    while time.time() - start < duration and sim.is_running():
        time.sleep(0.01)
    sim.set_base_velocity(0.0, 0.0)


# ---------------------------------------------------------
#  MAIN SIMULATION LOGIC
# ---------------------------------------------------------

def main():
    sim = StretchMujocoSimulator()

    print("Starting simulator...")
    sim.start(headless=False)

    if not sim.is_running():
        print("Failed to start simulator.")
        return

    # Go to home position
    sim.home()
    time.sleep(1.0)

    # -----------------------------------------------------
    # AVOID RED CYLINDER, THEN REACH BLUE BLOCK
    # -----------------------------------------------------

    # 1) Move sideways away from the red cylinder
    print("Step 1: Turn RIGHT 90° (face sideways away from obstacle)")
    turn_right(sim, speed=1.0, duration=1.8)

    print("Step 2: Move sideways FAR AWAY from obstacle")
    drive_forward(sim, speed=0.7, duration=2.0)   # <-- BIG sideways shift

    print("Step 3: Turn LEFT to face toward blue block")
    turn_left(sim, speed=1.0, duration=1.3)

    print("Step 4: Drive FORWARD toward blue block")
    drive_forward(sim, speed=1.0, duration=11.0)

    print("Step 5: Turn LEFT slightly to adjust alignment")
    turn_left(sim, speed=1.0, duration=0.6)

    print("Step 6: Final approach to blue block")
    drive_forward(sim, speed=0.7, duration=1.3)

    print("Navigation complete. Robot should now be close to the blue object.")
    input("Press ENTER to exit...")
    sim.stop()


if __name__ == "__main__":
    main()
