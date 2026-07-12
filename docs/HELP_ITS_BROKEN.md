

# Help its Broken
## First, identify where ROS is finding the packages:
```bash
ros2 pkg prefix bhl_st3215_driver
ros2 pkg prefix berkeley_biped_pkg
ros2 pkg prefix lilgreen_description
```
Also inspect the active prefix paths:

```bash
echo "$AMENT_PREFIX_PATH" | tr ':' '\n'
echo "$CMAKE_PREFIX_PATH" | tr ':' '\n'
```
# Check whether the new workspace captured the old underlay

grep -RnsE \
  'berkeley_ros2_ws|bhl_st3215_driver|berkeley_biped_pkg|lilgreen_description' \
  ~/littlegreen_ros2_ws/install/setup.* \
  ~/littlegreen_ros2_ws/install/_local_setup_util_* \
  2>/dev/null
``` 
# If that prints an old path such as:

```bash
/home/scott/berkeley_ros2_ws/install
then the LittleGreen workspace was built on top of the old overlay. Removing a line from ~/.bashrc will not fix the generated setup files; the workspace needs a clean rebuild from a clean ROS environment.
# Clean rebuild without inherited overlays
```
# Start a sterile shell:

```bash
env -i \
  HOME="$HOME" \
  USER="$USER" \
  LOGNAME="$LOGNAME" \
  SHELL=/bin/bash \
  TERM="${TERM:-xterm}" \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  LANG="${LANG:-en_US.UTF-8}" \
  bash --noprofile --norc
```
# Inside that clean shell:

```bash
source /opt/ros/humble/setup.bash

cd ~/littlegreen_ros2_ws
rm -rf build install log

export ONNXRUNTIME_DIR="$HOME/libs/onnxruntime-linux-aarch64-1.22.0"

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro humble \
  -r -y

colcon build \
  --symlink-install \
  --event-handlers console_direct+
```
# Then test before sourcing anything else:

```bash
source ~/littlegreen_ros2_ws/install/setup.bash

ros2 pkg prefix lgh_st3215_driver
ros2 pkg prefix littlegreen_biped_pkg
ros2 pkg prefix littlegreen_description
```
# The old names should now fail with “package not found”:

```bash
ros2 pkg prefix bhl_st3215_driver
ros2 pkg prefix berkeley_biped_pkg
ros2 pkg prefix lilgreen_description
```

Exit the sterile shell:

```bash
exit

```

Then close the original terminal and open a new one.

# Check the LittleGreen environment file

It should source only ROS Humble and the new workspace:

```bash
cat ~/.config/littlegreen/ros2_env.sh
```

Search all common startup files for old references:

```bash
grep -RnsE \
  'berkeley_ros2_ws|bhl_st3215|berkeley_biped|lilgreen_description' \
  ~/.bashrc \
  ~/.profile \
  ~/.bash_profile \
  ~/.bash_login \
  ~/.config \
  /etc/profile \
  /etc/bash.bashrc \
  /etc/profile.d \
  2>/dev/null
```
You can keep the old workspace archived; its presence on disk is harmless. The important requirements are that it is not sourced and that the current LittleGreen install/setup.bash was not generated with the old workspace as an underlay.
