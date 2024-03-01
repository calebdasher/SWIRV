# SWIRV
SWIRV (Shallow Water Intelligent River Vehicle), is a project meant to autonomously explore small creeks, streams, and rivers. As the name inplies, these bodies of water are shallow, and full of hazards such as rocks, sticks, and waterfalls, which SWIRV must detect and avoid, while exploring and taking video footage of the creek.
SWIRV is an ongoing WCU Capstone project, which is a senior year engineering project.
The project requires various software packages, and its currently unfinished. This is a repository of build scripts and instructions needed to load the sotware necessary for SWIRV onto an NVIDIA Jetson Nano B01.

What currently works, are the files needed to wirelessly control SWIRV using two ESP32's linked together with ESPNOW.
These files are wireless_transmit_esc_commands.ino and forward_reverse_servo.ino
The command file is a python file called joystick_control_C6.py
These programs were written for python 3.12, and Arduino IDE running the alpha version of ESP32 3.0 core library, on two ESP32-C6 Devkits.

The pictures are various screenshots taken during the development process while attempting to get stereoscopic vision, and lidar working.
The gps folder contains some gps sample code, which currently works, but probably won't end up as is in the final project.
The documentation folder contains build scripts and instructions for installing and configuring each module that is currently being tested and developed for SWIRV.

This project is planning to use ROS2, and currently running ROS2 Foxy, in order to develop a series of nodes and topics nessary to integrate all of SWIRV's sensors into a coherent control system necessary for autonomous navigation.
