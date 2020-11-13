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
This package contains the waypoint updater node: [`waypoint_updater.py`](./ros/src/waypoint_updater/waypoint_updater.py). The purpose of this node is to update the target velocity property of each waypoint based on traffic light and (eventually) obstacle data. This node will subscribe to the `/base_waypoints`, `/current_pose`, `/obstacle_waypoint`, and `/traffic_waypoint` topics, and publish a list of waypoints ahead of the car with target velocities to the `/final_waypoints` topic.

The `/current_pose` topic provides the vehicle's current position, and `/base_waypoints` provides a complete list of waypoints the car will be following.

#### [`/ros/src/twist_controller/`](./ros/src/twist_controller)
The vehicle is equipped with a drive-by-wire (dbw) system, meaning that the throttle, brake, and steering have electronic control. This package contains the files that are responsible for control of the vehicle: the node [`dbw_node.py`](./ros/src/twist_controller/dbw_node.py) and [`twist_controller.py`](./ros/src/twist_controller/twist_controller.py), along with a PID controller and lowpass filter used in the implementation. The `dbw_node` subscribes to the `/current_velocity` topic along with the `/twist_cmd` topic to receive target linear and angular velocities. Additionally, this node will subscribe to `/vehicle/dbw_enabled`, which indicates if the car is under dbw or driver control. This node will publish throttle, brake, and steering commands to the `/vehicle/throttle_cmd`, `/vehicle/brake_cmd`, and `/vehicle/steering_cmd` topics.

#### [`/ros/src/tl_detector/`](./ros/src/tl_detector)
This package contains the traffic light detection node: [`tl_detector.py`](./ros/src/tl_detector/tl_detector.py). This node takes in data from the `/image_color`, `/current_pose`, and `/base_waypoints` topics and publishes the locations to stop for red traffic lights to the `/traffic_waypoint` topic.

---
## Details of the Packages

Most of the modifications made to the packages have been driven by the information provided in the Project's classroom. In the following paragraph for each of them these references will be provided, together with details on any specific adptation/modification or feature development.

### _Waypoint Updater_
The content from Udacity's classroom can be found [here](https://www.youtube.com/watch?time_continue=1&v=6GIFyUzhaQo&feature=emb_logo) and [here](https://www.youtube.com/watch?v=2tDrj8KjIL4&feature=emb_logo).

* This [node](./ros/src/waypoint_updater) publishes a list of waypoints at a given frequency. The number of waypoints and the frequency have been changed with respect to what provided initially, to allow reliable execution against the characteristics of the system used for testing (Docker installation on 2017 MacBook Pro, 3.1 GHz, 16 GB RAM). The actual values depend from the use case and situation:
  * In case of NO traffic lights detection I could run up to a list of **100** waypoints updated at **50** Hz.
  * In case of traffic lights detection enabled (i.e. camera enabled in the sim), these numbers had to be reduced to **70** waypoints at **30** Hz
* At every updatethe node will:
  * Identify the closest amongst the base waypoints;
  * Identify the farthest waypoint based on the index of the closest and the number of waypoints to publish;
  * Verify wehther or not a stop line is located between the closest and farthest waypoint;
  * If no stop line is in sight just publish the identifed subset of waypoints;
  * If a stop line IS in sight, define a decelerating profile based on the distance from the stop line, and publish a list of waypoints enabling that deceleration.

### _Twist Controller_
The content from Udacity's classroom can be found [here](https://www.youtube.com/watch?v=kdfXo6atphY&feature=emb_logo).

* This [node](./ros/src/twist_controller) publishes a triad of commands in throttle, steering and brake at **50** Hz **IF** the Drive By Wire (DBW) functionality is enabled. 
  * Note that the frequency at which these commands are published shoud _not_ be changed (see also the video material).
* Every 1/50th of second the node will:
  * Collect the current value for the DBW enabled state, the current velocity and the current linear and angular velocity commands;
  * If the DBW is enabled execute the following actions:
    * Filter the velocity through a low-pass filter;
    * Calculate the steering command through a yaw-controller (pre-provided in the original code);
    * Calculate the throttle using a PID loop on the velocity error;
    * Estimate whether or not some braking action is needed and calculate it based on the vehicle characteristics (mass, wheel radius).

### _Traffic Lights Detector_ (No Classifier)
The content from Udacity's classroom can be found [here](https://www.youtube.com/watch?v=oTfArPhstQU&feature=emb_logo).

The actual design of the traffic lights detector [node](./ros/src/tl_detector) can be split in two steps: in the first one (documented in the video from the classroom) the actual state of the light (Red/Green/Yellow) is not inferred by a classifier analysing the image, but collected as one of the outputs of the simulator. At this stage the code can be modified to:

* Identify whether or not there's a traffic light close to the car;
  * "Close" is defined as: within a given number of waypoints, defined as a constant (in this code, set = **100**). 
* In that case, analyse the image of the camera and provide its state.
* In case of a red light, publish the position of the stop line, so that the Waypoint Updater can adapt the velocity profile accordingly (see section above).

A video showing the behaviour of the code at this stage can be seen in the following video:

[![Capstone No Class](http://img.youtube.com/vi/uvXvaQE95Zo/0.jpg)](https://youtu.be/uvXvaQE95Zo "Capstone No Class")

### Training of a Classifier
Once arrived at this stage, the next step is the training of a classifier capable of recognizing the state of a traffic light when the vehicle is close to it. 

In order to do so I have firstly gone through a [lab](https://github.com/udacity/CarND-Object-Detection-Lab) provided by Udacity to practice on the subject. I have then decided to extend the lab itself to document my work and its results: my updated version can be found [here](https://github.com/russom/CarND-Object-Detection-Lab).

Summarizing, I obtained a model based on the SSD Mobilnet one availble in the [TensorFlow 1 Object Detection Model Zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf1_detection_zoo.md), capable of classifying traffic lights in the Green/Red/Yellow classes. In order to do so I created a specific image dataset and followed the instruction available in the  [Object Detection API with TF1 Git Repo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf1.md) and in this series of [Blog posts](https://medium.com/@WuStangDan/step-by-step-tensorflow-object-detection-api-tutorial-part-1-selecting-a-model-a02b6aabe39e). The results of the training where pretty satisfactory.

**However**, there is a significant issue with the final result, related to the fact that I could NOT manage to set up a training environment compatible with the Udacity running one.

The fundamental problem is that the Udacity target environment makes reference to some libraries that are not supported anymore (see also the notes in the [README](README.md) about the various `requirements` files). More specifically, the Udacity environment targets ***TF 1.3***, while the V1 API available at the moment I'm writing this (mid-November 2020) makes use of ***TF 1.15.2***.
I tried several options, inlcuding running the [training scripts](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf1_training_and_evaluation.md) from the Udacity workspace, but all of them failed either because of the version of Tensor Flow (which would make the syntax of the scripts incompatible) or because of the version of Python (which would prevent them from running at all).

The main consequences of this are:
* The model _cannot_ run in the Udacity environment as is: as explained in the [README](README.md) an updated version of the `requirements` file has to be used to update TensorFlow;
* Even with the environment updated as such, the model _cannot_ find the right version of the CUDA libraries to make use of the GPU (the 1.15 training environment leverages CUDA 10 while the 1.3 target environment uses CUDA 8). As a consequence, the model resorts to use the CPU, which creates perfomances' issues.





