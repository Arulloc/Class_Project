Simulation of a 6-DOF Kinova Gen3 arm with a Robotiq 2F-85 gripper in Gazebo Classic (ROS 2 Humble), for capturing a rotating target. The arm has simulated motors and runs under Earth gravity.

## Setup (once per VM)

```bash
git clone https://github.com/Arulloc/Class_Project.git ~/class_project
bash ~/class_project/scripts/setup_vm.sh
```

The script asks for the `common` account's sudo password once.

## Running the simulation

**Terminal 1: Gazebo and the arm**

```bash
cd ~/class_project
source install/setup.bash
export GAZEBO_MODEL_PATH=$HOME/class_project/models:$GAZEBO_MODEL_PATH
ros2 launch launch/kinova_with_state.launch.py sim_gazebo:=true sim_ignition:=false dof:=6 launch_rviz:=false gripper:=robotiq_2f_85
```

**Terminal 2: the moving target** (after Gazebo has opened)

```bash
cd ~/class_project
source install/setup.bash
ros2 launch rotating_target target.launch.py mode:=spin
```

Get updates later with `cd ~/class_project && git pull`, then rerun `bash scripts/setup_vm.sh` to rebuild.

## What the setup script installs and why

| Item | Why | What it does |
| --- | --- | --- |
| ROS signing key (only if expired) | An expired key makes apt reject the ROS server | Lets ROS packages install |
| `ros-humble-ros2-control`, `ros-humble-ros2-controllers`, `ros-humble-gazebo-ros2-control` | The VM has no motor system | Simulated motors that hold the arm against gravity and move it |
| `ros-humble-ros-gz-bridge` | Kinova's launch file always starts it | Nothing in our setup (it is for Ignition, the newer Gazebo) |
| Kinova `ros2_kortex`, commit `116d87a` (humble branch) | Official arm model and simulation launch file | `kortex_description` (arm model), `kortex_bringup` (launch) |
| Robotiq `ros2_robotiq_gripper` (aalmrad fork), commit `e58bcda` | Gripper model; the version Kinova lists for Humble | `robotiq_description` (2F-85 model) |
| `rotating_target` (this repo) | Our moving target | Spawns the target and moves it |

## Arm launch settings

| Setting | Value | Why |
| --- | --- | --- |
| `sim_gazebo` | `true` | Use Gazebo Classic, the simulator on the VM |
| `sim_ignition` | `false` | Ignition (newer Gazebo) is not installed; its default is `true` |
| `dof` | `6` | The lab arm has 6 joints |
| `launch_rviz` | `false` | Gazebo already shows the arm; saves memory |
| `gripper` | `robotiq_2f_85` | Gripper on the lab arm (to confirm) |

`launch/kinova_with_state.launch.py` is Kinova's `kortex_sim_control.launch.py` with one change: Gazebo starts with `worlds/lab.world`, which adds the `gazebo_ros_state` plugin the target needs.

## Target settings

| Setting | Default | Meaning |
| --- | --- | --- |
| `mode` | `orbit` | `spin`: turns on the spot, handle goes round in a circle (turntable). `orbit`: whole cube moves in a circle. `both`: both at once |
| `spin_period` | `3.0` | Seconds per full turn in spin mode |
| `orbit_period` | `5.0` | Seconds per lap in orbit mode |
| `radius` | `0.10` | Orbit radius in metres |
| `x`, `y`, `z` | `0.5`, `0.0`, `0.20` | Target centre position in metres, relative to the arm base |

The target is a 10 cm cube with a loop handle 13 cm from its centre and an 8 cm AprilTag (36h10, ID 0) on top.

## Expected errors (not problems)

- `Failed loading controller twist_controller`: Kinova's velocity controller exists only on the real arm.
- `Failed to activate controller : robotiq_gripper_controller`: the gripper is passive in simulation, because neither Kinova nor Robotiq supports it in Gazebo Classic. Capture is judged geometrically in simulation.

## Folders

| Folder | Contents |
| --- | --- |
| `launch/` | `kinova_with_state.launch.py`: arm launch using our world |
| `models/` | `rotating_target`: the target; `target_APRILTAG_36H10_id0`: course tag model |
| `scripts/` | `setup_vm.sh` |
| `src/rotating_target/` | ROS package that spawns and moves the target |
| `worlds/` | `lab.world`: Earth gravity plus the `gazebo_ros_state` plugin |
| `src/ros2_kortex`, `src/ros2_robotiq_gripper` | Downloaded by the script (not stored in this repo) |
| `build/`, `install/`, `log/` | Made by `colcon build` (not stored in this repo) |

Do not run `sudo apt upgrade` on the course VM unless a lab says to.
