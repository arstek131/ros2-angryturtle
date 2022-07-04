# Angryturtle
First assignment for Robotics course @ USI 21/22.

## Prerequisites
- Python 3
- ROS2 (Galactic)
- Turtlesim

## Details
ROS2 node that controls a turtle in **turtlesim**:

- Turtle is able to write 'USI'
- When any other turtle gets closer than 2 meters to it at any given time, the turtle becomes angry, stops writing and starts pursuing the offender
- Angry turtle does not aim directly towards its target, but instead tries to look ahead m meters in front of the offender to intercept it and then eliminate it
- After the elimination of the offender, the angry turtle moves back to the initial position and restarts its writing behavior, ignoring anyother turtle on the way back
- When all the turtles have been killed, the angry turtle writes 'USI' for the last time

## Building from Source
Make sure to meet the Prerequisites

Clone the repository in your workspace.

The execution can be started simply using the following command:

```
  $ ros2 launch angry_turtle angry_turtle.launch.py
```

The file `angry_turtle.launch.py` is a launch file that contains the instruction to configure the two nodes required: the `turtlesim_node` and the node `angry_turtle_node.py` that contains the implemented controller.

Alternatively, it is possible to start the simulation as follows:

In one terminal launch the turtlesim node

In the oder terminal type:

```
  $ ros2 run angry_turtle angry_turtle_node
```

