# Strawberry_stacker

**Theme of the project**

The project's is an abstraction of a strawberry farm. It is the harvesting scenario in the strawberry farm where the harvesters are plucking the strawberry selectively which are ready for harvesting and packing them into small boxes. The harvesters usually separate the premium quality strawberries and pack them in a RED coloured box whereas the standard quality ones are packed in a BLUE coloured box. The boxes then have to be placed on respective trucks (red boxes on red truck and blue boxes on blue truck) to ship them further. The entire process is carried out by a team of multiple autonomous drones with the hep of tools such as PX4 Autopilot ecosystem for controlling the UAV, Gazebo simulator, a robotics simulator, where the simulated farm and UAV’s will reside, and ROS for integrating various aspects of autonomy required in the solution.

**Software Specifications**

1.  Ubuntu 20.04

    The operating system on which all the softwares run is Ubuntu 20.04. Since the theme is run entirely in Gazebo 11 simulation, Ubuntu 20.04 is the only supported operating system

2. ROS Noetic

    Robot Operating System (ROS) is the framework which is used to integrate the components in the theme.

3. Gazebo 11

    The entire theme is implemented inside the Gazebo simulation environment
    The official version supported in the theme is Gazebo 11.xx.
    Gazebo 11 is tightly integrated with ROS Noetic and so it comes pre-installed when ros-noetic-desktop-full is installed

4. Python

    Python is the language in which the participant’s programs are written
    The version of Python supported with ROS Noetic is Python 3
    Hence all the python programs interfacing with ROS Noetic framework is written in Python 3. Python 3 comes preinstalled with ROS Noetic
    
5. Px4 autopilot
    
    PX4 is an open source flight control software for drones and other unmanned vehicles. The project provides a flexible set of tools for drone developers to share     technologies to create tailored solutions for drone applications.
