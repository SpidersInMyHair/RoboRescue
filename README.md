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


Funtions Explained
-------------------
``debug()``

    print("Scan: ");
    print(" OpenPaths: ");
    for i in range(0, len(my_waypoints)):
        my_waypoints[i].printOpenings();
    print("");
    print(" ExploredPaths: ");
    for i in range(0, len(my_waypoints)):
        my_waypoints[i].printExplored();
    print("");

This prints out some important information for debugging. For example, it prints out what paths have been scanned at every waypoint. It shows a list of the paths that are open and those that have been explored for that waypoint.

``checkManualExit()``

    if (button.check_buttons(buttons=['left','right'])):
        stopMotors();
        exit();
    return True;
    
Simply checks if both the left and right button on the EV3 brick are pressed simultaneously. If they are, the program exits. 

``getGlobalDirection(heading)``

    reducedHeading = heading % 360;
    if (reducedHeading >= 315 or reducedHeading < 45):
        reducedHeading = 0;
    if (reducedHeading >= 45 and reducedHeading < 135):
        reducedHeading = 90;
    if (reducedHeading >= 135 and reducedHeading < 225):
        reducedHeading = 180;
    if (reducedHeading >= 225 and reducedHeading < 315):
        reducedHeading = -90; 
    return reducedHeading;
    
This function takes in any 'heading' or angle as an argument. It will then return the value of that angle as either 0, -90, 90 or 180 degrees. For example, passing in a value of 720 degrees will simply return 0. This is because 720 degrees is 360 degrees, which is 0 degrees.
Why is this done?
This is done because we want the robot to call forward by only 0 degrees. We don't want the robot to record an opening at 0 degrees and then at some later point as 360 degrees.
