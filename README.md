[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Overview
---

This is the final Capstone Project submitted as part of the Udacity Self-Driving Car Nanodegree.

For it the goal is to write code implementing ROS nodes for a SW responsible to drive autonomously a vehicle either in a simulated or real environment, identifying traffic lights and stopping at them when needed.

The source code for this project is submitted as part of this Git repo (in the [`ros/src`](/ros/src) folder). A detailed explanation is provided in a separate [writeup](Capstone_writeup.md), that documents also the results obtained.  

---

Dependencies
---
First of all, this project involves the Udacity Term 3 Simulator which can be downloaded [here](https://github.com/udacity/CarND-Capstone/releases).

To run the simulator on Mac/Linux, you might have to first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

In terms of other dependencies, they would depend on the kind of installation selected. There are, in fact, two options to run the code: through a native installation on a Linux environment or using a Docker container.

Please use **one** of the two installation options, either native **or** Docker installation. Consider that most of the testing done for the code happened with the **Docker** installation.

**NOTE** on websocket set up: 

For both installations you might have to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For this you can use the scripts and follow the instructions provided in previous projects (like the [Path Planning](https://github.com/russom/CarND-P7-Path-Planning)) one. 

Docker Installation
---

1. [Install Docker](https://docs.docker.com/engine/installation/)
2. From the root folder of this repo, build the Docker container

```bash
   docker build . -t capstone
```

3. From the same place, run the Docker file

```bash
   docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

After the last command the system will prompt you in the running container, showing you something like this:

```
   root@xxxxxxxxxxxx:/capstone/ros#
```

**NOTE** on `Dockerfile` modifications:

The first run of the `docker build` command after cloning the original Udacity repo generated the following error message on my system:

```bash
   Command "python setup.py egg_info" failed with error code 1 in /tmp/pip-build-BTDEEY/markdown/
   You are using pip version 8.1.1, however version 20.1.1 is available.
   You should consider upgrading via the 'pip install --upgrade pip' command.
   The command '/bin/sh -c pip install -r requirements.txt' returned a non-zero code: 1
```

To resolve it, I updated the [`dockerfile`](./Dockerfile) as explained in the message itself.

**NOTE** on `requirements.txt`

The original `requirements.txt` file as found in the [original Udacity repo](https://github.com/udacity/CarND-Capstone) lists all the libraries needed for the code to run both on a virtual environment as well as on the real Udacity self-driving car (Carla). However I found 2 ussues with that list:

1) When creating this repo on my own, the Git [Dependabot](https://dependabot.com/) complained heavily about some of them requiring updates for security patches;
1) In running the code built using the original file, after ticking the `Camera` checkbox in the simulator I received the following error message:

```bash
   Traceback (most recent call last):
     File "src/gevent/greenlet.py", line 766, in gevent._greenlet.Greenlet.run
     File "/usr/local/lib/python2.7/dist-packages/socketio/server.py", line 651, in _handle_event_internal
       r = server._trigger_event(data[0], namespace, sid, *data[1:])
     File "/usr/local/lib/python2.7/dist-packages/socketio/server.py", line 680, in _trigger_event 
       return self.handlers[namespace][event](*args)
     File "/capstone/ros/src/styx/server.py", line 52, in image 
       bridge.publish_camera(data)
     File "/capstone/ros/src/styx/bridge.py", line 185, in publish_camera
       image_message = self.bridge.cv2_to_imgmsg(image_array, encoding="rgb8")
     File "/opt/ros/kinetic/lib/python2.7/dist-packages/cv_bridge/core.py", line 248, in cv2_to_imgmsg 
       img_msg.height = cvim.shape[0]
   IndexError: tuple index out of range
   2020-06-23T17:10:31Z <Thread at 0x7f63002d0890: <bound method Server._handle_event_internal of <socketio.server.Server object at 0x7f63224e9f10>>(<socketio.server.Server object at 0x7f63224e9f10>, '1d195dbc19384fa6b14ac8c5e608525b', [u'image', {u'image': u'/9j/4AAQSkZJRgABAQAAAQABAA, '/', None)> failed with IndexError
```

Based on the experience from [another student](https://github.com/Horki/CarND-Capstone) this also seemed to be due to the version of one of the libraries (`Pillow`).

As a consequence of both points, I have updated [`requirements.txt`](./requirements.txt) to load newer versions of the the libraries. The original libraries are still listed in the file if needed, they're just commented out.

I have mostly used the Docker installation to test the code up to the introduction of a classifier for the traffic lights. This task was indeed too heavy for my machine, and so I resorted to make use of a native installation, provided by Udacity through a dedicated project workspace. 

Native Installation
---

As an alternative to Docker I have used a specific ***Udacity Workspace***, i.e. a fully equipped virtual environment with all the proper dependencies installed, capable to run aginst a GPU. As a guide to set up a native environment from scratch, however, the following instructions can be followed:

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space
* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
  
Ater having cloned the project repo, you would have to install the python dependencies. For this use case, they are defined in a file I called [`requirements_ws.txt`](./requirements_ws.txt):

```bash
  cd CarND-Capstone
  pip install -r requirements_ws.txt
```

**NOTE** on `requirements_ws.txt`

The [`requirements_ws.txt`](./requirements_ws.txt) is a copy of the original `requirements.txt` file as found in the [original Udacity repo](https://github.com/udacity/CarND-Capstone) EXCEPT for `tensorflow` and `numpy`that have been updated. This is a consequence of the issue I found in training and running a classifier starting from the current (at the moment I write this) [TensorFlow V1 object detection API](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf1.md). I could not find a way to do that without updating TF to 1.15 (ideally 1.15.2). This also required the update of `numpy`.

More details are available in the [writeup](Capstone_writeup.md)

### Other library/driver information
Outside of `requirements_ws.txt`, here is information on other driver/library versions used in the simulator and Carla:

Specific to these libraries, the simulator grader and Carla use the following:

|        | Simulator | Carla  |
| :-----------: |:-------------:| :-----:|
| Nvidia driver | 384.130 | 384.130 |
| CUDA | 8.0.61 | 8.0.61 |
| cuDNN | 6.0.21 | 6.0.21 |
| TensorRT | N/A | N/A |
| OpenCV | 3.2.0-dev | 2.4.8 |
| OpenMP | N/A | N/A |

---

Running the code
---

Whether you are using the Docker or native installation, you can run the code issuing the same commands. In case of native installation, however, first of all you will have to move to the `ros` folder (as seen before the Docker one prompts you there directly). 

Then, in both cases, type:

```bash
   catkin_make
   source devel/setup.sh
   roslaunch launch/styx.launch
```

After that you can start the simulator.

**NOTE** on clean build: 

In some cases it might be desirable to be sure that the SW running is coming from a "clean: build. To achieve that, it's needed to delete the `ros/build` and `ros/devel` folders that are generate by the `catkin_make` command. In order to simplify that process I have added to the `ros` folder a [`clean_build.sh`](./ros/clean_build.sh) script that does just that and then builds and sources the code. To use it and then run the code, from the `ros` folder just type:

```bash
   source clean_build.sh
   roslaunch launch/styx.launch
```
