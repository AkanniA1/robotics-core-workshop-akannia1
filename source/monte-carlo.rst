Clever Gambling in Monaco
=====================================

**Theme:** *Clever Gambling in Monaco* — this week introduces 
**Monte Carlo Localization (MCL)**, where probability meets robotics. 
Robots "gamble" with particles to find out where they really are.

Focus
-----

- Understand the principles of **Monte Carlo Localization (MCL)**.
- Learn how particle filters estimate a robot’s pose.
- Explore the connection between probability, motion models, and sensor models.
- Implement MCL in Python and compare it with ROS 2’s **AMCL** package.

Learning Flow
-------------

1. **Theory**
   - The localization problem: *"Where am I?"*
   - Monte Carlo method: random sampling for probabilistic estimation.
   - Particle filters:
     
     - Initialization
     - Motion update
     - Sensor update
     - Resampling
   - Why it’s called *Monte Carlo* (link to casino roulette analogy).

2. **Practical / Code**
   - Implement a custom **Python MCL** with NumPy:
     
     - Initialize particles in free space
     - Propagate with a noisy motion model
     - Weight particles based on sensor likelihood
     - Resample to focus on probable poses
   - Visualize the particle cloud converging on the robot’s location.

3. **ROS 2 Integration**
   - Use **Nav2 AMCL** for real-world localization.
   - Launch AMCL with a 2D occupancy map.
   - Visualize particle clouds in RViz2.
   - Compare results from your custom Python implementation.

4. **Assignment**
   - Write a Python implementation of MCL.
   - Run **Nav2 AMCL** with TurtleBot3 (or another robot) in Gazebo.
   - Push results, code, and a short README to GitHub:
     
     - Explanation of MCL
     - Your implementation
     - Screenshots of ROS 2 AMCL in action

Takeaway
--------

By the end of this week, students will understand how robots **use probability to localize**. 
They will see how random "gambling" with particles transforms into accurate localization, 
both in a **custom Python implementation** and in **ROS 2 AMCL**.  

This week brings to life the connection between probability theory, code, 
and real-world robotics — demonstrating that sometimes, **gambling is clever**.