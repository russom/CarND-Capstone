## Capstone Project
The goal of this project is the implementation, in Python, of ROS nodes belonging to the SW for an autonomus vehicle, responsible of driving it through either a simulated or real environment. One of the node will include a classifier for traffic lights, and the SW will stop the vehicle in case of a red one. 

---
## System Architecture
The content of this section is mostly taken from the Udacity Project's classroom, and here reported for clarity and completeness.

First of all, the following picture shows a system architecture diagram detailing the ROS nodes and topics used in the project:


<p align="center">
  <img width="1200" height="640" src="./imgs/SystemArch.png">
</p>

## Code Structure
The code for the project is contained entirely within the [`/ros/src/`](./ros/src) directory. Within this directory, you will find the following ROS packages, that have been modified:

### [`/ros/src/tl_detector/`](./ros/src/tl_detector)
This package contains the traffic light detection node: [`tl_detector.py`](./ros/src/tl_detector/tl_detector.py). This node takes in data from the `/image_color`, `/current_pose`, and `/base_waypoints` topics and publishes the locations to stop for red traffic lights to the /traffic_waypoint topic.

The `/current_pose` topic provides the vehicle's current position, and `/base_waypoints` provides a complete list of waypoints the car will be following.
