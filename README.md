# RoboRescue
The robot that searches through a maze, collects a target and then exits the maze.

This is currently done through a waypoint system. A waypoint is created at every new intersection.
A waypoint object contains 3 important things:
  1. A list of intersection directions that are 'open' paths.
  2. A list of intersection directions that have been explored.
  3. The intersection direction that the robot entered from.
  
The robot will select an open path at a waypoint that has not been explored. It will then travel down this path until a new 
intersection is found. 
When an intersection is reached that has no unexplored open paths (for example a dead-end in the maze), the robot will recall to 
the previous waypoint. It will continue to do this until it has returned to a waypoint where there is still something to be explored.
