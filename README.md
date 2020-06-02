# autonomas-navigation
Thesis project on autonomas indoor navigation of 3-wheels robot

Hardware used: Raspberry PI 4, Asus Xtion PRO Live, gyro G-521

Software used: Lubuntu 16.04 (compiled with ROS kinetic), ROS Kinetic

For running abstacles avoidance algorithm:

1. roslaunch opnneni2_launch openni2.launch
2. rosrun depthimage_to_laserscan depthimage_to_laserscan image:=/camera/depth/image_raw
3. roslaunch navigation gyro_on.launch
4. rosrun navigation avoid_obstacles.py
