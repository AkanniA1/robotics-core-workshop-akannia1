Mission ROSsible
====================================

**Theme:** *Mission ROSsible* — this week introduces students to the 
**Robot Operating System 2 (ROS 2)**, the middleware that powers modern robotics. 
It may look complex, but with the right guidance, every mission in ROS 2 is possible.

.. image:: _static/ros.gif
   :alt: Example GIF
   :align: center

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



.. ROS 2 Nodes
.. ===========

.. A **node** is a fundamental building block in ROS 2. It is a process that performs computation. 
.. Nodes can communicate with each other by publishing and subscribing to topics, providing and using services, 
.. or sending and receiving actions.

.. Example of a simple ROS 2 node in Python:

.. .. code-block:: python

..     #!/usr/bin/env python3

..     import rclpy
..     from rclpy.node import Node

..     class MyNode(Node):
..         """A simple ROS 2 node that logs a message every second."""

..         def __init__(self):
..             super().__init__("py_test")
..             self.count = 0
..             self.get_logger().info("Node has started: hello world")
..             self.create_timer(1.0, self.timer_callback)

..         def timer_callback(self):
..             self.get_logger().info(f"hello {self.count}")
..             self.count += 1

..     def main(args=None):
..         rclpy.init(args=args)
..         node = MyNode()
..         rclpy.spin(node)
..         node.destroy_node()
..         rclpy.shutdown()

..     if __name__ == "__main__":
..         main()

.. This node is named ``py_test`` and logs a greeting message every second, incrementing a counter each time.

.. ---

.. ROS 2 Service
.. =============

.. A service provides request/response communication.

.. **Example: A service that adds two integers.**

.. **Service server:**

.. .. code-block:: python

..     #!/usr/bin/env python3
..     from example_interfaces.srv import AddTwoInts
..     import rclpy
..     from rclpy.node import Node

..     class AddTwoIntsService(Node):
..         """A simple AddTwoInts service."""

..         def __init__(self):
..             super().__init__('add_two_ints_service')
..             self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_callback)

..         def add_callback(self, request, response):
..             response.sum = request.a + request.b
..             self.get_logger().info(f'Request: {request.a} + {request.b} = {response.sum}')
..             return response

..     def main(args=None):
..         rclpy.init(args=args) 
..         node = AddTwoIntsService()
..         rclpy.spin(node)
..         rclpy.shutdown()

..     if __name__=="__main__" :
..         main()

.. **Service client:**

.. .. code-block:: python

..     #!/usr/bin/env python3
..     from example_interfaces.srv import AddTwoInts
..     import rclpy

..     def main():
..         rclpy.init()
..         node = rclpy.create_node('add_two_ints_client')
..         client = node.create_client(AddTwoInts, 'add_two_ints')

..         while not client.wait_for_service(timeout_sec=1.0):
..             node.get_logger().info('Service not available, waiting...')

..         req = AddTwoInts.Request()
..         req.a = 5
..         req.b = 3

..         future = client.call_async(req)
..         rclpy.spin_until_future_complete(node, future)
..         print(f'Result: {future.result().sum}')

..         node.destroy_node()
..         rclpy.shutdown()

..     if __name__ == "__main__":
..         main()

.. ---

.. ROS 2 Action
.. ============

.. An **action** is a long-running goal that can provide feedback and be preempted.

.. **Example: A simple Fibonacci action server.**

.. **Action server:**

.. .. code-block:: python

..     #!/usr/bin/env python3
..     from example_interfaces.action import Fibonacci
..     from rclpy.action import ActionServer
..     import rclpy
..     from rclpy.node import Node

..     class FibonacciActionServer(Node):
..         """A Fibonacci action server."""

..         def __init__(self):
..             super().__init__('fibonacci_action_server')
..             self._action_server = ActionServer(
..                 self,
..                 Fibonacci,
..                 'fibonacci',
..                 self.execute_callback)

..         async def execute_callback(self, goal_handle):
..             self.get_logger().info('Executing Fibonacci action...')

..             feedback_msg = Fibonacci.Feedback()
..             sequence = [0, 1]
..             for i in range(1, goal_handle.request.order):
..                 sequence.append(sequence[i] + sequence[i-1])
..                 feedback_msg.partial_sequence = sequence
..                 goal_handle.publish_feedback(feedback_msg)

..                 # Simulate async work
..                 await rclpy.task.Future()

..             goal_handle.succeed()
..             result = Fibonacci.Result()
..             result.sequence = sequence
..             return result

..     def main(args=None):
..         rclpy.init(args=args) 
..         node = FibonacciActionServer()
..         rclpy.spin(node)
..         rclpy.shutdown()

..     if __name__=="__main__" :
..         main()

.. The **client** sends a goal (e.g., to compute a sequence of length 10) and receives feedback 
.. with partial results until the sequence is complete.

.. ---
