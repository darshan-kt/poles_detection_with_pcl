
ROS 2 Navigation and Pole Detection Guide
This repository documents the process of setting up and practicing ROS 2 navigation components (Mapping, SLAM, AMCL, NAV2) step-by-step, as well as developing a novel pole detection algorithm using 3D LiDAR point cloud data.

Tutorial Source
The instructions in this repository are based on the Husarion ROS 2 Tutorial.

Installation
Follow the steps below to set up the environment:

### Create a workspace and clone the repository
"""
mkdir -p rosbot_ws/src
cd rosbot_ws
git clone https://github.com/husarion/rosbot_xl_ros src/
"""

### Update and install dependencies
sudo apt-get update
sudo apt install ros-dev-tools

### Import hardware and simulation repositories
vcs import src < src/rosbot_xl/rosbot_xl_hardware.repos
vcs import src < src/rosbot_xl/rosbot_xl_simulation.repos

### Initialize rosdep and install dependencies
sudo rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y

### Build the workspace
colcon build --symlink-install

### Add the workspace to the bashrc for easy sourcing
echo 'source ~/rosbot_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc

### Practicing ROS 2 Navigation
### Running the Simulator
### Launch the simulation robot:
ros2 launch rosbot_xl_gazebo simulation.launch.py

### Teleop control (keyboard control):
ros2 run teleop_twist_keyboard teleop_twist_keyboard

### Visualize in Rviz:
rviz2

### In Rviz, add the RobotModel plugin and set its topic to /robot_description.
### Mapping (SLAM Toolbox)
### Launch the simulation robot:
ros2 launch rosbot_xl_gazebo simulation.launch.py


### Start SLAM Toolbox:
ros2 launch robot_navigation slam.launch.py use_sim_time:=true

### Visualize in Rviz:
rviz2

### Teleop control:
ros2 run teleop_twist_keyboard teleop_twist_keyboard

### Save the map:
ros2 run nav2_map_server map_saver_cli -f map

### Loading a Saved Map
### Start the Map Server:
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=map.yaml -p use_sim_time:=true

### Bring up the Map Server Lifecycle Node:
ros2 run nav2_util lifecycle_bringup map_server

### Localization (AMCL)
### Launch the simulation robot:
ros2 launch rosbot_xl_gazebo simulation.launch.py

### Run AMCL:
ros2 launch robot_navigation amcl.launch.py use_sim_time:=true

### Visualize in Rviz:
rviz2

### Add ParticleCloud and select the /particle_cloud topic.
### Use the 2D Pose Estimate tool to set the robot's initial pose.
### Navigation
### Launch the simulation robot:
ros2 launch rosbot_xl_gazebo simulation.launch.py

### Visualize in Rviz:
rviz2

### Ensure the map topic is selected before launching NAV2.
### Run NAV2:
ros2 launch robot_navigation navigation.launch.py use_sim_time:=true

### Pole Detection and Midpoint Calculation
### This repository includes a custom algorithm to detect poles and calculate their midpoints using 3D LiDAR point cloud data.

# Steps:
# Launch the simulation robot with a custom poles environment:
ros2 launch rosbot_xl_gazebo simulation_poles.launch.py

### Visualize in Rviz:
rviz2

### Add PointCloud2 and RobotModel plugins.
### Run the pole detection algorithm:
ros2 run find_poles_pkg pole_detection

### Calculate the midpoint:
ros2 run find_poles_pkg midpoint

### Teleop the robot:
ros2 run teleop_twist_keyboard teleop_twist_keyboard

Notes
Ensure the robot and Rviz are correctly configured for visualization.
For NAV2, the map topic is published only once. Run Rviz and select the map topic before launching NAV2.
Follow the Husarion tutorial for detailed guidance and troubleshooting.