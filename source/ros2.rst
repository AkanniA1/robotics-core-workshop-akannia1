Mission ROSsible
====================================

**Theme:** *Mission ROSsible* — this week introduces students to the 
**Robot Operating System 2 (ROS 2)**, the middleware that powers modern robotics. 
It may look complex, but with the right guidance, every mission in ROS 2 is possible.

Focus
-----

- Learn the purpose and architecture of **ROS 2**.
- Understand how nodes, topics, services, and actions work together.
- Build and run basic ROS 2 publishers and subscribers.
- Teleoperation with TurtleBot3

Learning Flow
-------------

1. **Theory**
   - What is ROS 2 and why is it essential in robotics?
   - Nodes, topics, services, and actions: the communication backbone.
   - ROS 1 vs ROS 2: key improvements (DDS, real-time, security).
   - Packages, workspaces, and launch files.

2. **Practical / Code**
   - Install and set up a ROS 2 workspace.
   - Write a simple publisher and subscriber in Python.
   - Use command-line tools:
     
     - `ros2 topic list`, `ros2 topic echo`
     - `ros2 node info`
   - Create a custom message and send data between nodes.

3. **Simulation / Visualization**
   - Launch a demo robot in **Gazebo** or **RViz2**.
   - Publish velocity commands to move the robot.
   - Visualize sensor data streams in real-time.

4. **Assignment**
   - Create a ROS 2 package with:
     
     - A publisher node
     - A subscriber node
   - Test communication between nodes.
   - Push code to GitHub with a README explaining how to build and run the package.

Takeaway
--------

By the end of this week, students will be able to **confidently navigate ROS 2**, 
create simple nodes, and communicate between them.  
They will understand how ROS 2 acts as the *operating system of robotics*, 
making every mission — no matter how complex — truly **ROSsible**.