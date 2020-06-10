		
# Introduction 
Enable human-robot controls by untrained operators through Mixed Reality Devices & Services.

- Mixed Reality Devices: Build intuitive control of the robot and facilitation of human robot collaboration through HoloLens2 (Gaze, Gesture, Voice) and other mobile Mixed Reality devices, thus enabling humans and robots to work together.

- Mixed Reality Services: Build co-localization and shared environmental understanding capabilities, enabling multi-robot control in a shared coordinate system.

# Overview

The [spot_ros_interface](./spot_ros_interface/README.md) ROS package provides a ROS interface to the Spot API. 

# Getting Started
## 1.	Installation process

1) Set up WSL (Ubuntu 20.04): https://docs.microsoft.com/en-us/windows/wsl/install-win10

2) Download and install Spot SDK: https://github.com/boston-dynamics/spot-sdk/blob/master/docs/python/quickstart.md#getting-the-code

3) Set up ROS Noetic in WSL:
- Configure your Ubuntu repositories to allow "restricted," "universe," and "multiverse". Follow instructions [here](https://help.ubuntu.com/community/Repositories/CommandLine).
Remember to `sudo apt-get update` after this.


- Set up sources.list
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
- Set up keys
```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
If you experience issues connecting to the keyserver, you can try substituting `hkp://pgp.mit.edu:80`

If you still experience issues like:
 `gpg: keyserver receive failed: No dirmngr`
 or
 `gpg: can't connect to the agent: IPC connect call failed`  (see [here](https://github.com/microsoft/WSL/issues/5125) and [here](https://stackoverflow.com/questions/46673717/gpg-cant-connect-to-the-agent-ipc-connect-call-failed)), ensure `dirmngr` is installed:
 ```
 sudo apt-get install dirmngr
 ```
 If this still does not solve the problem:
```
sudo apt remove gpg
sudo apt install gnupg1
```
You should now be able to re-run the key setup command above.
- Install
```
sudo apt update
sudo apt install ros-noetic-desktop-full
```
This will take some time. Go grab a coffee :coffee:.

Once that is done, let us verify the installation. You must source this script in every bash terminal you use ROS in:
```
source /opt/ros/noetic/setup.bash
```
To automatically source it whenever you open a new terminal, add it to your .bashrc:
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

To verify the installation, run `roscore` in the terminal. This should start a roslaunch server without any errors. Feel free to kill the server (Ctrl+C) now.

Congratulations! You have just set up ROS. Now let's install the  the dependencies.

## 2.	Software dependencies


## 3.	Latest releases
## 4.	API references
# Build and Test
TODO: Describe and show how to build your code and run the tests. 
# Contribute
[ROS package guidelines](https://github.com/ethz-asl/mav_tools_public/wiki/How-to-Write-a-ROS-Package)
