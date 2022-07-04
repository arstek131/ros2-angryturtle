# Angryturtle
First assignment for Robotics course @ USI 21/22.

## Prerequisites
- Python 3
- ROS 2 (Galactic)
- Turtlesim

## Details
ROS2 node that controls a turtle in **turtlesim**:

- Turtle is able to write 'USI'
- When any other turtle gets closer than 2 meters to it at any given time, the turtle becomes angry, stops writing and starts pursuing the offender
- Angry turtle does not aim directly towards its target, but instead tries to look ahead m meters in front of the offender to intercept it and then eliminate it
- After the elimination of the offender, the angry turtle moves back to the initial position and restarts its writing behavior, ignoring anyother turtle on the way back
- When all the turtles have been killed, the angry turtle writes 'USI' for the last time
