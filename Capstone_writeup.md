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

Summarizing, I obtained a model based on the SSD Mobilnet one availble in the [TensorFlow 1 Object Detection Model Zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf1_detection_zoo.md), capable of classifying traffic lights in the Green/Red/Yellow classes. In order to do so I created a specific image dataset and followed the instruction available in the  [Object Detection API with TF1 Git Repo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf1.md) and in this series of [Blog posts](https://medium.com/@WuStangDan/step-by-step-tensorflow-object-detection-api-tutorial-part-1-selecting-a-model-a02b6aabe39e). As it can be seen from the lab, the results of the training where pretty satisfactory.

**However**, there is a significant issue with the final result, related to the fact that I could NOT manage to set up a training environment compatible with the Udacity running one.

The fundamental problem is that the Udacity target environment makes reference to some libraries that are not supported anymore (see also the notes in the [`README`](README.md) about the various `requirements` files). More specifically, the Udacity environment targets ***TF 1.3***, while the V1 API available at the moment I'm writing this (mid-November 2020) makes use of ***TF 1.15.2***.
I tried several options, inlcuding running the [training scripts](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf1_training_and_evaluation.md) from the Udacity workspace, but all of them failed either because of the version of Tensor Flow (which would make the syntax of the scripts incompatible) or because of the version of Python (which would prevent them from running at all).

The main consequences of this are:
* The model _cannot_ run in the Udacity environment as is: as explained in the [`README`](README.md) an updated version of the `requirements` file has to be used to update TensorFlow;
* Even with the environment updated as such, the model _cannot_ find the right version of the CUDA libraries to make use of the GPU (the 1.15 training environment leverages CUDA 10 while the 1.3 target environment uses CUDA 8). As a consequence, the model resorts to use the CPU, which creates perfomances' issues. An example of the CUDA messages thrown is:

```
   2020-11-11 09:15:39.459606: I tensorflow/stream_executor/cuda/cuda_gpu_executor.cc:983] successful NUMA node read from SysFS had negative value (-1), but there must be at least one NUMA node, so returning NUMA node zero
   2020-11-11 09:15:39.460614: I tensorflow/core/common_runtime/gpu/gpu_device.cc:1618] Found device 0 with properties: 
   name: Tesla K80 major: 3 minor: 7 memoryClockRate(GHz): 0.8235
   pciBusID: 0000:00:04.0
   2020-11-11 09:15:39.460996: W tensorflow/stream_executor/platform/default/dso_loader.cc:55] Could not load dynamic library 'libcudart.so.10.0'; dlerror: libcudart.so.10.0: cannot open shared object file: No such file or directory; LD_LIBRARY_PATH: /home/workspace/CarND-Capstone/ros/devel/lib:/opt/ros/kinetic/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu:/opt/carndcapstone/cuda-8.0/extras/CUPTI/lib64/:/opt/carndcapstone/cuda-8.0/lib64/
   2020-11-11 09:15:39.461231: W tensorflow/stream_executor/platform/default/dso_loader.cc:55] Could not load dynamic library 'libcublas.so.10.0'; dlerror: libcublas.so.10.0: cannot open shared object file: No such file or directory; LD_LIBRARY_PATH: /home/workspace/CarND-Capstone/ros/devel/lib:/opt/ros/kinetic/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu:/opt/carndcapstone/cuda-8.0/extras/CUPTI/lib64/:/opt/carndcapstone/cuda-8.0/lib64/
   2020-11-11 09:15:39.461700: W tensorflow/stream_executor/platform/default/dso_loader.cc:55] Could not load dynamic library 'libcufft.so.10.0'; dlerror: libcufft.so.10.0: cannot open shared object file: No such file or directory; LD_LIBRARY_PATH: /home/workspace/CarND-Capstone/ros/devel/lib:/opt/ros/kinetic/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu:/opt/carndcapstone/cuda-8.0/extras/CUPTI/lib64/:/opt/carndcapstone/cuda-8.0/lib64/
   2020-11-11 09:15:39.461964: W tensorflow/stream_executor/platform/default/dso_loader.cc:55] Could not load dynamic library 'libcurand.so.10.0'; dlerror: libcurand.so.10.0: cannot open shared object file: No such file or directory; LD_LIBRARY_PATH: /home/workspace/CarND-Capstone/ros/devel/lib:/opt/ros/kinetic/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu:/opt/carndcapstone/cuda-8.0/extras/CUPTI/lib64/:/opt/carndcapstone/cuda-8.0/lib64/
   2020-11-11 09:15:39.462339: W tensorflow/stream_executor/platform/default/dso_loader.cc:55] Could not load dynamic library 'libcusolver.so.10.0'; dlerror: libcusolver.so.10.0: cannot open shared object file: No such file or directory; LD_LIBRARY_PATH: /home/workspace/CarND-Capstone/ros/devel/lib:/opt/ros/kinetic/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu:/opt/carndcapstone/cuda-8.0/extras/CUPTI/lib64/:/opt/carndcapstone/cuda-8.0/lib64/
   2020-11-11 09:15:39.462585: W tensorflow/stream_executor/platform/default/dso_loader.cc:55] Could not load dynamic library 'libcusparse.so.10.0'; dlerror: libcusparse.so.10.0: cannot open shared object file: No such file or directory; LD_LIBRARY_PATH: /home/workspace/CarND-Capstone/ros/devel/lib:/opt/ros/kinetic/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu:/opt/carndcapstone/cuda-8.0/extras/CUPTI/lib64/:/opt/carndcapstone/cuda-8.0/lib64/
   2020-11-11 09:15:39.462924: W tensorflow/stream_executor/platform/default/dso_loader.cc:55] Could not load dynamic library 'libcudnn.so.7'; dlerror: libcudnn.so.7: cannot open shared object file: No such file or directory; LD_LIBRARY_PATH: /home/workspace/CarND-Capstone/ros/devel/lib:/opt/ros/kinetic/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu:/opt/carndcapstone/cuda-8.0/extras/CUPTI/lib64/:/opt/carndcapstone/cuda-8.0/lib64/
   2020-11-11 09:15:39.462999: W tensorflow/core/common_runtime/gpu/gpu_device.cc:1641] Cannot dlopen some GPU libraries. Please make sure the missing libraries mentioned above are installed properly if you would like to use GPU. Follow the guide at https://www.tensorflow.org/install/gpu for how to download and setup the required libraries for your platform.
Skipping registering GPU devices...
```

### _Traffic Lights Detector_ (Classifier included)
The model used for classification can be found in the [`data/ssd_mobilenet_2018_fine_tuned_model`](./data/ssd_mobilenet_2018_fine_tuned_model) folder. In there I saved (for the sake of completeness) all the output of the export script provided with the TF API: the actual file used for the clasiification is `frozen_inference_graph.pb`.

The TL detector [node](./ros/src/tl_detector/tl_detector.py) was modified mosltly in the `get_light_state()` function (starting at line 121), that in turns makes use of files stored in the [`light_classification`](./ros/src/tl_detector/light_classification) folder. Specifically

* [`tl_classifier.py`](./ros/src/tl_detector/light_classification/tl_classifier.py) defines the `TLClassifier` class, which hosts methods to process images;
* [`helpers.py`](./ros/src/tl_detector/light_classification/helpers.py) contains a couple of helper functions to load the model or convert the color scheme of the images.

Most of these functions are refactoring of instructions that can be found in the [Jupyter notebook](https://github.com/russom/CarND-Object-Detection-Lab/blob/master/CarND-Object-Detection-Lab.ipynb) that is part of the Object Detection lab; the layout itself is the same of the one proposed by [this](https://github.com/Horki/CarND-Capstone) previous project.

I have collected some examples of the classifier running in the code against the simulator in the Udacity workspace in this video:

[![Classifier Tests](http://img.youtube.com/vi/J1qMqC8sMXg/0.jpg)](https://youtu.be/J1qMqC8sMXg "Capstone No Class")

These tests have been collecting driving the vehcile manually along the simulator track and enabling the camera when close to a traffic lights. It can be immediately noticed how the classifier is capable of identifying the lights properly, but it takes too long to do that to allow a closed loop implementation. The main reason that I could find for this is the one mentioned in the previos paragraph: the model cannot run against the GPU and the CPU cannot provide the capabilities needed.

Few more notes:

* In order to limit the impact of everytrhing else other than the classifier, in the tests shown in the video I have further reduced the number of waypoints defining the horizon for both the Waypoint Updater and the Traffic Lights Detector nodes: I put both of them = 25.
* I have added few lines [3:9] in the `import` section of [`tl_classifier.py`](./ros/src/tl_detector/light_classification/tl_classifier.py). They prevent the deprecation and CUDA warnings to be shown on the screen to avoid polluting the shell, but they can be commented out if needed. The lines are:

```
   # Remove warnings from Cuda
   import os
   os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3' 
   #
   import tensorflow as tf
   # Remove warings from tf
   tf.get_logger().setLevel('ERROR')
```

---
## Conclusions

* The code provided in this repo was capable of driving a vehicle in the Udacity Capstone Project simulator, defining and following a set of waypoints and implementing a strategy to decelerate and stop at red traffic lights, in the case in which the state of the light (Red/Green/Yellow) is provided by the simulator itself.
* A classifier based on the SSD Mobilenet model was built, capable of succesfully identifying the state of a traffic light (as Red/Green/Yellow) based on an acquired image. Open-loop tests on reference images are satisfactory.
* Unfortunately, it was not possible for me to succesfully include the classifier as part of the running code, to execute the classification in closed loop, while the vehicle is driving. This was due to the unavailability of a training environment compatible with the target execution one. 


