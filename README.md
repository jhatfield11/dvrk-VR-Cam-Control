# dvrk-VR-Cam-Control
This repository has code to allow control of a DaVinci research kit in simulation using a virtual reality rig. The Repository also contains methods to allow for autonomous camera control of the endoscope camera manipulator via kinematics. 

Running the repository is not dependent on a VR rig but may produce errors. To run the repository knowledge of Unity and ROS will be required. 

##  Structure of the Project 
* Unity-dvrk-VR
  * A unity project that contains scripts, URDF's scenes and prefabs to generate the PSM's ,MTM's and ECM manipulators in Unity and input from a VR rig
  * The virtual robots can be controlled by grabbing the robots in VR or attatching an end effector to the movement of a VR controller. 
  * Message generation and ROS communication are handled by the ROS-TCP-Connector. Two scripts are available publish and subscribe to JointState messages passed over the connector.
* ROS-dvrk-VR
  * A set of python scripts to connect with the unity environment, publish to the dvrk manipulators and perform autonomous camera control.
  * JSON files to run the console application with two patient side manipulators and the endoscope camera manipulator. 
  * ROS communication is handled by a TCP server from ROS-TCP-Connector. 
