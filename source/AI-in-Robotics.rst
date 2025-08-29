AI in Robotics 
=============================================

LLM → Navigation
--------------------

**Theme:** *Words to Waypoints* — this week we teach a robot to turn
natural-language commands (e.g., "go to the charging dock, then the kitchen")
into concrete **navigation goals** in ROS 2. We combine **LLMs** (for language
understanding) with **Nav2** (for motion) and a light **world ontology** that
maps words to places.

Focus
-----

- Understand how LLMs interpret intent, entities, and spatial relations.
- Design a **location ontology** (names → map poses/areas).
- Build a **command parser** (baseline rules + LLM fallback).
- Convert language into **Nav2 NavigateToPose** action goals.
- Run end-to-end in **Gazebo + Nav2**, with confirmation and safety checks.

Prerequisites
-------------

- A working Nav2 stack (e.g., TurtleBot3 in Gazebo) and a map (`.png/.yaml`).
- Basic ROS 2 Python (rclpy), actions, and topics.
- For LLM integration: any HTTP client + an LLM endpoint (cloud or local).  
  *(We provide a stub so you can swap in your provider.)*

Theory
------

- **Grounding**: linking language tokens (e.g., "kitchen", "left of table")
  to map frames, poses, or zones.
- **Intent & entities**: detect goals, sequences ("then"), and constraints
  ("avoid corridor", "slowly").
- **Ambiguity management**: confirm, ask clarifying questions, or fall back to
  safe defaults.
- **Pipelined design**:
  1) *NL → structured plan* (intent + waypoints),  
  2) *plan → geometry* (poses in `map` frame),  
  3) *geometry → Nav2 actions*.