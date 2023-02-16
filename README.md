# **Mobile Robotics Project Report** 

**Group 36:** <br>
Vincent Gherold <br>
Vicente Carbon <br>
Yifei Luo <br>
Emeric de Galembert <br>


Project of robotics that implements: vision, global navigation, local navigation and filtering.

Here is a video that demonstrates the 4 aspects of the project.

Vision: track the position of the robot, detect the position of the obastacles
Global navigaiton: solves the traveling salesman problem to connect all the goals (green dots) and then come back at the initial position
Local naviagation: distance sensores are used to avoid abstacles that arise in the path
Filtering: a kalman filter is implemented to track the position of the robot. It merges the information coming from the vision and from the odometry. Therefore, if the the camera is obstructed, the position estimation will only be based on odometry.


You can open report.ipynb to know more about it
