# setup_vm.sh: sets up the Kinova Gen3 simulation on an EECS 4421 VM.
# Run from anywhere:  ~/class_project/scripts/setup_vm.sh
# Asks for the sudo password once. Safe to run again.

set -e                                   # stop at the first error
cd "$(dirname "$0")/.."                  # go to the project folder
echo "Project folder: $(pwd)"

# 1. ROS signing key: refresh only if it has expired
echo "== 1. Checking the ROS package key =="
if sudo apt update 2>&1 | grep -q EXPKEYSIG; then
  echo "ROS key expired: downloading the current one"
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  sudo apt update
fi

# 2. System packages: motor system for the simulated arm, plus the
#    Ignition bridge that Kinova's launch file always starts
echo "== 2. Installing ROS packages =="
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control ros-humble-ros-gz-bridge

# 3. Third-party code, pinned to the exact versions we tested
echo "== 3. Downloading Kinova and Robotiq code =="
mkdir -p src
if [ ! -d src/ros2_kortex ]; then
  git clone https://github.com/Kinovarobotics/ros2_kortex.git src/ros2_kortex
fi
git -C src/ros2_kortex checkout 116d87ae595c791ba7ae32201b88693a3ba02324    # humble branch, 2026-02-06

if [ ! -d src/ros2_robotiq_gripper ]; then
  git clone https://github.com/aalmrad/ros2_robotiq_gripper.git src/ros2_robotiq_gripper
fi
git -C src/ros2_robotiq_gripper checkout e58bcda6a73bfc7570ffcc4f53067284e6fa3cab    # source listed in ros2_kortex.humble.repos, 2025-03-14

# 4. Build only what we use; skip Kinova's real-arm driver
echo "== 4. Building =="
source /opt/ros/humble/setup.bash
colcon build --packages-select kortex_description kortex_bringup robotiq_description --packages-ignore kortex_driver kortex_api

echo ""
echo "Setup complete. In each new terminal run:"
echo "  cd ~/class_project && source install/setup.bash"
