#!/usr/bin/env bash
# Build abb_hardware_interface + abb_rws_client and run their tests on ROS 2
# Jazzy, in a throwaway container, against the pinned dependencies in
# abb.repos. This is the merge gate for this fork (there is no hosted CI).
#
#   scripts/test_jazzy.sh [image]     # default: ros:jazzy-ros-base
#
# The tests drive the hardware interface against a fake robot controller that
# speaks EGM over UDP on the container's localhost (see
# abb_hardware_interface/test/fake_egm_robot.hpp); no robot or network needed.
set -euo pipefail
repo=$(cd "$(dirname "$0")/.." && pwd)
image=${1:-ros:jazzy-ros-base}
docker run --rm -v "$repo":/src/abb_ros2:ro "$image" bash -c '
set -eo pipefail
apt-get update -qq && apt-get install -y -qq python3-vcstool git > /dev/null
mkdir -p /ws/src && cd /ws
cp -a /src/abb_ros2 src/abb_ros2 && rm -rf src/abb_ros2/.git
vcs import src < src/abb_ros2/abb.repos > /dev/null
source /opt/ros/jazzy/setup.bash
rosdep update -q --rosdistro jazzy > /dev/null
rosdep install -y -q --rosdistro jazzy --ignore-src --from-paths \
  src/abb_libegm src/abb_librws src/abb_egm_rws_managers src/abb_ros2_msgs \
  src/abb_ros2/abb_hardware_interface src/abb_ros2/abb_rws_client > /dev/null
colcon build --packages-up-to abb_hardware_interface abb_rws_client \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --event-handlers console_cohesion- > /dev/null
echo "build OK"
colcon test --packages-select abb_hardware_interface abb_rws_client \
  --return-code-on-test-failure --event-handlers console_cohesion- > /dev/null || true
colcon test-result --all | grep -E "test_[a-z_]*\.gtest\.xml|lint" ; colcon test-result --verbose
'
