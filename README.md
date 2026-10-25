Class_project (EECS 4421)

Simulation of a 6-DOF Kinova Gen3 arm with a Robotiq 2F-85 gripper in Gazebo, for capturing a rotating target. The arm has simulated motors and runs under Earth gravity.

git clone https://github.com/Arulloc/Class_Project.git ~/class_project
~/class_project/scripts/setup_vm.sh

The script asks for the `common` account's sudo password once. Then, in a **new** terminal:

cd ~/class_project
source install/setup.bash
ros2 launch kortex_bringup kortex_sim_control.launch.py sim_gazebo:=true sim_ignition:=false dof:=6 launch_rviz:=false gripper:=robotiq_2f_85

Get updates later with `cd ~/class_project && git pull`.

## What the setup script installs and why

| Item | Why | What it does |
| ROS signing key (only if expired) | An expired key makes apt reject the ROS server | Lets ROS packages install |
| `ros-humble-ros2-control`, `ros-humble-ros2-controllers`, `ros-humble-gazebo-ros2-control` | The VM has no motor system | Simulated motors that hold the arm against gravity and move it |
| `ros-humble-ros-gz-bridge` | Kinova's launch file always starts it | Nothing in our setup (it is for Ignition, the newer Gazebo) |
| Kinova `ros2_kortex`, commit `116d87a` (humble branch) | Official arm model and simulation launch file | `kortex_description` (arm model), `kortex_bringup` (launch) |
| Robotiq `ros2_robotiq_gripper` (aalmrad fork), commit `e58bcda` | Gripper model; the version Kinova lists for Humble | `robotiq_description` (2F-85 model) |

## Launch settings

| Setting | Value | Why |
| `sim_gazebo` | `true` | Use Gazebo, simulator on the VM |
| `sim_ignition` | `false` | Ignition (newer Gazebo) is not installed; its default is `true` |
| `dof` | `6` | The lab arm has 6 joints |
| `launch_rviz` | `false` | Gazebo already shows the arm; saves memory |
| `gripper` | `robotiq_2f_85` | Gripper on the lab arm (to confirm) |

## Expected errors (not problems)

- `Failed loading controller twist_controller`: velocity controller exists only on the real arm.
- `Failed to activate controller : robotiq_gripper_controller`: gripper is passive in simulation, because neither Kinova nor Robotiq supports it in Gazebo. Capture determined geometrically in sim.

## Folders

| `scripts/` | `setup_vm.sh` |
| `worlds/` | `lab.world`: Earth-gravity world |
| `src/` | Kinova and Robotiq code, downloaded by the script (not stored in this repo) |
| `build/`, `install/`, `log/` | Made by `colcon build` (not stored in this repo) |
