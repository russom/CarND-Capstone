[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Overview
---

This is the final Capstone Project submitted as part of the Udacity Self-Driving Car Nanodegree.

For it the goal is to write code implementing ROS nodes for a SW responsible to drive autonomously a vehicle either in a simulated or real environment, identifying traffic lights and stopping at them when needed.

The source code for this project is submitted as part of this Git repo (in the [`ros/src`](/ros/src) folder). A detailed explanation is provided in a separate [writeup](Capstone_writeup.md), that documents also the results obtained.  

Dependencies
---
First of all, this project involves the Udacity Term 3 Simulator which can be downloaded [here](https://github.com/udacity/CarND-Capstone/releases)

To run the simulator on Mac/Linux, you might have to first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

In terms of other depednencies, they would depend on the kind of installation selected. There are, in fact, two options to run the code: through a native installation on a Linux environment or using a Docker container.

Please use **one** of the two installation options, either native **or** docker installation. Consider that most of the testing done for the code happened with th **Docker** installation.

**NOTE** on websocket set up: For both installations you might have to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For this you can use the scripts and follow the instructions provided in previous projects (like the [Path Planning](https://github.com/russom/CarND-P7-Path-Planning) one. 

### _Docker Installation_

1. [Install Docker](https://docs.docker.com/engine/installation/)
2. From the root folder of this repo, build the docker container

```bash
   docker build . -t capstone
```

3. From the same place, run the docker file

```bash
   docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

After the last command the system will prompt you in the running container:

```
   root@0410f09901dc:/capstone/ros#
```

### _Native Installation_

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
  
Finally, after having cloned the project repo, you would have to install the python dependencies defined in [`requirements.txt`](./requirements.txt):

```bash
  cd CarND-Capstone
  pip install -r requirements.txt
```


---



### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```


```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

### Other library/driver information
Outside of `requirements.txt`, here is information on other driver/library versions used in the simulator and Carla:

Specific to these libraries, the simulator grader and Carla use the following:

|        | Simulator | Carla  |
| :-----------: |:-------------:| :-----:|
| Nvidia driver | 384.130 | 384.130 |
| CUDA | 8.0.61 | 8.0.61 |
| cuDNN | 6.0.21 | 6.0.21 |
| TensorRT | N/A | N/A |
| OpenCV | 3.2.0-dev | 2.4.8 |
| OpenMP | N/A | N/A |

We are working on a fix to line up the OpenCV versions between the two.
