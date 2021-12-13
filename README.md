# EECS 106A Final Project - Search and Rescue

Node: move arm in predetermined path for scan
Service: closest AR tag to cone
Service: turtlebot AR tag

Node: AR tags
Publish: list of AR tags in view with coordinates

Node: plan_next_target
Looks at all AR tags in view, calculates minimum manhattan distance to goal pose in World frame
Calculates transform between target AR tag and turtlebot in world frame
subscribe: AR tags, last stopped position of turtlebot, turtlebot boolean status
publish: intermediate target pose in turtlebot frame


Node: move baxter arm in orientation restricted path from start to goal
stop when sees new AR tag
subscribe: turtlebot boolean status, list of AR tags in view
Client to: move arm in predetermined path for scan

Node: turtlebot_move
subcribe: next ar_target
publish: boolean status (moving or stopped)

Node: turtlebot_seek


## Turtlebot Setup: Do this before launching demo_baxter.launch
source devel/setup.bash

ssh turtlebot@pink.local

export ROS_MASTER_URI=http://archytas.local:11311

## 1 terminal
roslaunch turtlebot_bringup minimal.launch

## 1 terminal
roslaunch turtlebot_bringup 3dsensor.launch


## Archytas Setup:

source devel/setup.bash
./baxter.sh archytas.local

### 1 terminal
rosrun baxter_tools enable_robot.py -e

rosrun baxter_search baxter_cam.py -o right_hand_camera -r 1280x800 --exposure 5

rosrun baxter_interface joint_trajectory_action_server.py

### 1 terminal
roslaunch baxter_moveit_config demo_baxter.launch load_robot_description:=true

### 1 terminal
roslaunch baxter_search right_arm_ar_track.launch

---

## Phase 1
rosrun baxter_search find_turtlebot_and_goal.py

rosrun baxter_search initial_sweep.py


## Phase 2
rosrun turtlebot_nav turtlebot_hop.py

rosrun baxter_search guiding_path.py

rosrun baxter_search guide_coordinator.py

## Phase 3

rosrun turtlebot_nav turtlebot_final_countdown.py

# Runs

rosrun baxter_search dummy.py # all 3 phases

rosrun baxter_search dummy2.py # phase 2 and 3
