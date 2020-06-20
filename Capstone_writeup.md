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
The code for the project is contained entirely within the [`/ros/src/`](./ros/src) directory. Within this directory, the following ROS packages have been modified:

#### [`/ros/src/waypoint_updater/`](./ros/src/waypoint_updater)
This package contains the waypoint updater node: [`waypoint_updater.py`](./ros/src/waypoint_updater/waypoynt_updater.py). The purpose of this node is to update the target velocity property of each waypoint based on traffic light and (eventually) obstacle data. This node will subscribe to the `/base_waypoints`, `/current_pose`, `/obstacle_waypoint`, and `/traffic_waypoint` topics, and publish a list of waypoints ahead of the car with target velocities to the `/final_waypoints` topic.

The `/current_pose` topic provides the vehicle's current position, and `/base_waypoints` provides a complete list of waypoints the car will be following.

#### [`/ros/src/twist_controller/`](./ros/src/twist_controller)
The vehicle is equipped with a drive-by-wire (dbw) system, meaning the throttle, brake, and steering have electronic control. This package contains the files that are responsible for control of the vehicle: the node [`dbw_node.py`](./ros/src/twist_controller/dbw_node.py) and [`twist_controller.py`](./ros/src/twist_controller/twist_controller.py), along with a pid and lowpass filter used in the implementation. The `dbw_node` subscribes to the `/current_velocity` topic along with the `/twist_cmd` topic to receive target linear and angular velocities. Additionally, this node will subscribe to `/vehicle/dbw_enabled`, which indicates if the car is under dbw or driver control. This node will publish throttle, brake, and steering commands to the `/vehicle/throttle_cmd`, `/vehicle/brake_cmd`, and `/vehicle/steering_cmd` topics.

#### [`/ros/src/tl_detector/`](./ros/src/tl_detector)
This package contains the traffic light detection node: [`tl_detector.py`](./ros/src/tl_detector/tl_detector.py). This node takes in data from the `/image_color`, `/current_pose`, and `/base_waypoints` topics and publishes the locations to stop for red traffic lights to the /traffic_waypoint topic.

---
## Details of the Packages

Most of the modifications made to the packages have been driven by the information provided in the Project's classroom. In the following paragraph for each of them these references will be provided, together with details on any specific adptation/modification or feature development.

### _Waypoint Updater_
Content from Uacity's callsroom [here](https://www.youtube.com/watch?time_continue=1&v=6GIFyUzhaQo&feature=emb_logo) and [here](https://www.youtube.com/watch?v=2tDrj8KjIL4&feature=emb_logo).

### _Twist Controller_
Content from Uacity's callsroom [here](https://www.youtube.com/watch?v=kdfXo6atphY&feature=emb_logo)

### _Traffic Lights Detector_
Content from Uacity's callsroom [here](https://www.youtube.com/watch?v=oTfArPhstQU&feature=emb_logo)
