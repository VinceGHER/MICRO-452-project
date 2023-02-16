# **Mobile Robotics Project Report** 


Project of robotics that implements: vision, global navigation, local navigation and filtering.

Here is a video that demonstrates the 4 aspects of the project.

Vision: track the position of the robot, detect the position of the obastacles
Global navigaiton: solves the traveling salesman problem to connect all the goals (green dots) and then come back at the initial position
Local naviagation: distance sensores are used to avoid abstacles that arise in the path
Filtering: a kalman filter is implemented to track the position of the robot. It merges the information coming from the vision and from the odometry. Therefore, if the the camera is obstructed, the position estimation will only be based on odometry.

[![Watch the video](https://img.youtube.com/vi/KVVF_peBu2s/default.jpg)](https://youtu.be/KVVF_peBu2s)

For a detailed explanation of the project, see the folder "Report.ipynb".

This project was realised in the scope of the class "Basics of mobile robotics" (MICRO-452) thaught by Mondada Francesco.
