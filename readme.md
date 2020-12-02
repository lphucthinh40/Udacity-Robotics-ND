# UDACITY ROBOTICS NANODEGREE PROJECT
This repo contains ROS nodes created for assignments from the Udacity's Robotics Software Nanodegree. Completed assignments:
1. **Build My World**: design a Gazebo world environment & custom models
2. **Go Chase It**: design and build a mobile robot that can detect and chase a white-color ball
3. **Where Am I**: use Adaptive Monte Carlo Localization (AMCL) to localize a mobile robot

### Go Chase It
  - Design and build a mobile robot with camera & laser sensor
  - Place the robot in a custom-built Gazebo world
  - The robot is capable of chasing a ball that is within its field of vision

**Build & Run Instructions:**
- git clone this repo
- run `source devel/setup.bash`
- run `roslaunch my_robot world.launch`
- run `roslaunch ball_chaser ball_chaser.launch` to make the robot chase the white ball

### Where Am I
- Generate pgm map file from an existing Gazebo world using **pgm_map_creator** package
- Use Adaptive Monte Carlo Localization **amcl navigation** ROS package to localize the robot
- Fine tune amcl node to improve the accuracy of particle filter state estimation

**Build & Run Instruction:**
- git clone this repo
- run `source devel/setup.bash`
- run `roslaunch my_robot world.launch`
- open another terminal, run `roslaunch my_robot amcl.launch` to start localization
- (optional) open another terminal, run `rosrun teleop_twist_keyboard teleop_twist_keyboard.py` to control the robot with keyboard
