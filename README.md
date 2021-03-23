# Project Starling

The purpose of Starling is to update the control systems within the Bristol Robotics Laboratory Flight Arena in order to reduce the barrier to entry for those who wish to experiment with, and fly real drones.

This project takes recent advancements in cloud computing, namely containerisation (docker) and container orchestration (kubernetes) as the core framework to manage networking and drone deployment, whether virtual (using SITL) or on real drones running companion computers (e.g. raspberry pi).

However a key requirement of the system is to allow users to be able to operate drones without needing to know more about the udnerlying implementation.

This systems provides a number of key features.

- Primary support for ROS2 and MavLink
- Built in simulation stack based on Gazebo
- Quick transition from simulation to flying controllers on real drones.

Please refer to the documentation [TODO: add permanent link] for more detailed instructions and explanations of how to use this system.
