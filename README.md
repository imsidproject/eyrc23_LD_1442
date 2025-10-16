# eyrc23_LD_1442

Video Demonstration: [https://youtu.be/BTzbautauXM](https://youtu.be/BTzbautauXM)\
Project made for e-Yantra Robotics Competition 2023 organized by IIT Bombay.\
Theme: Luminosity Drone\
Autonomous drone that scans the arena for specific objects and reports their location and type.\
Implemented in:
- Simulation (branch: main)
- Hardware (branch: ros2)
  
**Robot Operating System** (ROS1 in simulation, ROS2 in hardware) was used.\
Computer Vision, Image Processing was done using OpenCV on the video frames captured from the onboard drone camera.
### Setup Details:
1. Drone with a whycon marker on top.
2. Overhead camera connected to laptop through USB.
3. Laptop running ROS.
4. Laptop connected to drone through Wifi.
5. Raspberry Pi inside drone running ROS.

### How it works:
1. Overhead camera captures video.
2. Whycon : a ROS node that processes the video and publishes the X,Y,Z coordinates of the detected whycon marker on the drone.
3. alien_finder/alien_finder.py: A ROS Node, that reads the coordinates and processes the video feed from the onboard drone camera. It publishes control signals to the drone (Throttle, Roll, Pitch, Yaw, and beep commands) that the drone obeys.
4. Raspberry Pi on the drone running ROS. It reads the control commands coming from laptop. It also publishes the video frames.
