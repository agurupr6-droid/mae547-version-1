# Stretch Mujoco - RRT Path Planning Simulation

[![Python 3.10](https://img.shields.io/badge/python-3.10-blue.svg)](https://www.python.org/downloads/release/python-31012/)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white)](https://github.com/pre-commit/pre-commit)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)

<img src="https://github.com/hello-robot/stretch_mujoco/raw/main/docs/images/stretch_mujoco.png" title="Stretch In Kitchen" width="100%">

This repository provides a simulation stack for the Stretch robot, built on [MuJoCo](https://github.com/google-deepmind/mujoco). The project includes position control for the arm, head, and gripper joints, velocity control for the mobile base, calibrated camera RGB + depth imagery, 2D spinning lidar scans, and more. This version includes RRT (Rapidly-exploring Random Tree) path planning algorithms for autonomous navigation with obstacle detection and avoidance.

## Features

- **RRT Path Planning**: Implementation of RRT algorithms for collision-free path planning
- **Obstacle Detection and Avoidance**: Real-time obstacle detection that allows the robot to change direction and navigate around obstacles
- **Robot Simulation**: Full Stretch robot simulation with MuJoCo physics engine
- **Position & Velocity Control**: Control arm, head, gripper, and mobile base
- **Camera & Sensor Data**: Access to RGB, depth imagery, and lidar scans
- **Visualizer Support**: Interactive visualizer or efficient headless mode

## Scenario Description (version_2 - Scenario 2)

In this scenario, the robot uses RRT path planning to navigate to a goal position. The robot continuously monitors its environment for obstacles. When an obstacle is detected, the robot changes its direction to avoid collision and continues navigating toward the goal. The simulation stops when the robot successfully reaches the goal state.

## Prerequisites

- Python 3.10 or higher
- Git (for cloning the repository)
- [uv](https://docs.astral.sh/uv/) package manager

## Setup Instructions

### 1. Install uv Package Manager

First, install `uv`, which is a fast Python package manager that will handle virtual environment creation and dependency management:

**On Windows (PowerShell):**
```powershell
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
```

**On macOS/Linux:**
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

After installation, restart your terminal or reload your shell configuration.

### 2. Clone the Repository

Clone this repository:

```bash
git clone <repository-url>
cd mae547-version-1
```

### 3. Set Up Virtual Environment and Install Dependencies

The `uv` package manager automatically creates and manages a virtual environment for you. Simply run:

```bash
uv sync
```

This command will:
- Create a virtual environment (`.venv` directory) automatically
- Install all project dependencies from `pyproject.toml`
- Install the project itself in editable mode

> **Note**: `uv` handles virtual environment creation automatically. You don't need to manually create one with `python -m venv` or `virtualenv`.

### 4. Verify Installation

To verify everything is set up correctly, you can check the installed packages:

```bash
uv pip list
```

## Running the Simulation

### Run RRT Path Planning Simulation

To run the main RRT path planning simulation:

```bash
uv run run_sim.py
```

This script will:
1. Start the simulator
2. Plan a collision-free path using RRT algorithm
3. Execute the path while detecting obstacles
4. Change direction when obstacles are detected
5. Stop at the goal position

Press `Ctrl+C` in the terminal to exit the simulation.

## Switching Between Scenarios

This repository contains multiple branches, each representing a different simulation scenario. You can switch between branches to access different scenarios:

### Available Scenarios

- **Scenario 1**: Available on the `version_3` branch
- **Scenario 2**: Available on the `version_2` branch (current branch)
- **Scenario 3**: Available on the `version_4` branch

### How to Switch Branches

1. **Check current branch:**
   ```bash
   git branch
   ```

2. **Switch to a different scenario branch:**
   
   For Scenario 1:
   ```bash
   git switch version_3
   ```
   
   For Scenario 2:
   ```bash
   git switch version_2
   ```
   
   For Scenario 3:
   ```bash
   git switch version_4
   ```

3. **After switching branches, reinstall dependencies:**
   ```bash
   uv sync
   ```
   
   > **Note**: Different branches may have different dependencies or code changes. It's recommended to run `uv sync` after switching branches to ensure all dependencies are properly installed.

4. **Run the simulation:**
   ```bash
   uv run run_sim.py
   ```

### Branch Differences

Each branch contains scenario-specific modifications:
- **version_2** (Scenario 2): Modified `run_sim.py` with scenario-specific parameters - robot detects obstacles, changes direction, and stops at goal
- **version_3** (Scenario 1): Modified `run_sim.py` with scenario-specific parameters - robot stops after colliding with obstacle
- **version_4** (Scenario 3): Modified `run_sim.py` and `scene.xml` with scenario-specific parameters

To see what files differ between branches, you can use:
```bash
git diff --name-status main <branch_name>
```

## Project Structure

```
mae547-version-1/
├── run_sim.py                    # Main RRT path planning simulation
├── stretch_mujoco/               # Main simulation package
│   ├── __init__.py
│   ├── config.py                 # Configuration settings
│   ├── stretch_mujoco_simulator.py  # Core simulator class
│   ├── utils.py                  # Utility functions
│   ├── mujoco_server.py          # MuJoCo server implementation
│   ├── mujoco_server_passive.py  # Passive viewer server
│   ├── mujoco_server_managed.py  # Managed viewer server
│   ├── mujoco_server_camera_manager.py  # Camera management
│   ├── mujoco_server_sensor_manager.py # Sensor management
│   ├── datamodels/               # Data model classes
│   │   ├── status_command.py
│   │   ├── status_stretch_camera.py
│   │   ├── status_stretch_joints.py
│   │   └── status_stretch_sensors.py
│   ├── enums/                    # Enumeration classes
│   │   ├── actuators.py
│   │   ├── stretch_cameras.py
│   │   └── stretch_sensors.py
│   └── models/                   # Robot and scene models
│       ├── scene.xml             # Main scene configuration
│       ├── stretch.xml           # Robot model definition
│       ├── docking_station.xml    # Docking station model
│       └── assets/               # 3D meshes, textures, and images
│           ├── *.obj             # 3D mesh files
│           ├── *.stl             # STL mesh files
│           └── *.png             # Texture images
├── pyproject.toml                # Project dependencies and configuration
├── uv.lock                       # Dependency lock file
├── README.md                     # This file
├── CHANGELOG.md                  # Version history
└── LICENSE                       # License file
```

## Troubleshooting

### Build Error on Linux

If you see a build error mentioning `evdev` on Linux, run:

```bash
sudo apt install python3-dev
```

### macOS Library Issues

On macOS, if `mjpython` fails to locate `libpython3.10.dylib` and `libz.1.dylib`, run:

```bash
# Reload your terminal/IDE to ensure UV environment variables are loaded
source .venv/bin/activate

# When libpython3.10.dylib is missing:
PYTHON_LIB_DIR=$(python3 -c 'from distutils.sysconfig import get_config_var; print(get_config_var("LIBDIR"))')
ln -s "$PYTHON_LIB_DIR/libpython3.10.dylib" ./.venv/lib/libpython3.10.dylib

# When libz.1.dylib is missing:
export DYLD_LIBRARY_PATH=/usr/lib:$DYLD_LIBRARY_PATH
```

### Virtual Environment Activation (Optional)

While `uv run` automatically uses the virtual environment, you can manually activate it if needed:

**On Windows (PowerShell):**
```powershell
.venv\Scripts\Activate.ps1
```

**On macOS/Linux:**
```bash
source .venv/bin/activate
```

## Writing Custom Code

You can use the `StretchMujocoSimulator` class to interact with the simulation:

```python
from stretch_mujoco import StretchMujocoSimulator

if __name__ == "__main__":
    sim = StretchMujocoSimulator()
    sim.start(headless=False)  # Opens MuJoCo viewer window
    
    # Robot control
    sim.stow()
    sim.home()
    sim.move_to('lift', 1.0)
    sim.set_base_velocity(0.3, -0.1)
    
    # Get status
    status = sim.pull_status()
    camera_data = sim.pull_camera_data()
    
    sim.stop()
```

> **Important**: Always use the `if __name__ == "__main__":` guard when writing scripts, as explained in the [Python multiprocessing documentation](https://docs.python.org/3/library/multiprocessing.html).

## Additional Resources

- [Changelog](./CHANGELOG.md) - Version history and changes

## Dependencies

Key dependencies (automatically installed with `uv sync`):
- `mujoco==3.2.6` - Physics simulation engine
- `opencv-python` - Computer vision
- `matplotlib>=3.10.1` - Plotting and visualization
- `hello-robot-stretch-urdf>=0.1.0` - Robot model definitions
- And more (see `pyproject.toml` for complete list)

## License

See [LICENSE](./LICENSE) file for details.

## Acknowledgments

The assets in this repository contain significant contributions from [Kevin Zakka](https://github.com/kevinzakka) and [Google Deepmind](https://github.com/google-deepmind), along with others in Hello Robot Inc. who helped in modeling Stretch in MuJoCo.
