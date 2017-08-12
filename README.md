# RoboRescue
The robot that searches through a maze, collects a target and then exits the maze.
Written by Rishad Mahbub & Jason Ma
For ENGG1000
Dated 2017S1

This is currently done through a waypoint system. A waypoint is created at every new intersection.
A waypoint object contains 3 important things:
  1. A list of intersection directions that are 'open' paths.
  2. A list of intersection directions that have been explored.
  3. The intersection direction that the robot entered from.
  
The robot will select an open path at a waypoint that has not been explored. It will then travel down this path until a new 
intersection is found. 
When an intersection is reached that has no unexplored open paths (for example a dead-end in the maze), the robot will recall to 
the previous waypoint. It will continue to do this until it has returned to a waypoint where there is still something to be explored.


Changelog
----------

 Added the color sensor checking for orange (only does this when robot is moving forward). If it detects orange, motors should stop
 robot should beep and RETURN_PHASE set to True.
 Added loop backtracking only if necessary. So does not backtrack a loop if every waypoint in that loop is explored.
 Added forward alignment as well as reverse alignment when a forward closed wall is available to use as reference for alignment.


Funtions Explained
-------------------
Explains what each of the functions do. Looking at the relevent functions in navigation.py while reading this will really help.


``debug()``

This prints out some important information for debugging. For example, it prints out what paths have been scanned at every waypoint. It shows a list of the paths that are open and those that have been explored for that waypoint.


``checkManualExit()``
    
Simply checks if both the left and right button on the EV3 brick are pressed simultaneously. If they are, the program exits. 


``getGlobalDirection(heading)``
   
This function takes in any 'heading' or angle as an argument. It will then return the value of that angle as either 0, -90, 90 or 180 degrees. For example, passing in a value of 720 degrees will simply return 0. This is because 720 degrees is 360 degrees, which is 0 degrees.
Why is this done?
This is done because we want the robot to call forward by only 0 degrees. We don't want the robot to record an opening at 0 degrees and then at some later point as 360 degrees.


``stopMotors()``
    
Immediately kills both the motors that are responsible for driving the wheels.


``turnSonar(position)``
    
This will turn the servo/medium motor that the sonar sensor is attatched to. 
It should only take in an argument of -1, 0, 1.
-1 will turn it 90 degrees to the left relative to the body of the robot.
0 will turn it forward in alignment with the body of the robot.
1 will turn it 90 degrees to the right relative to the body of the robot.


``resetSonar()``
    
Positions the sonar servo in the forward-facing direction and forces the sonar to be held in this position for a given time (SONAR_RESET_SLEEP). 


``moveForward(speed)``

When called, even once, it will instantly drive the motors forward at the given speed. The motors will continue to run at this speed until the motors are stopped or given a different instruction.
It will also the robot aligned in that direction. For example, uneven friction on the ground etc. will cause a forward moving
moving robot to eventually become angled. This function fights against that by powering down a wheel to counter this.


``turnRelative(degree)``

Given the degree argument, this function will turn the entire robot by that much.
A positive value will turn the robot clockwise. A negative value will turn the robot anti-clockwise.
Also notice that when the amount to turn becomes less than 45 (including for example, halfway through a 90 degree turn) then the robot will reduce its turn speed.
This slowing down of the turn speed as the robot gets closer to its target angle makes sure that the robot does not 'overshoot', as powering down a high-speed motor will take times and cause degrees of inaccuracy. 


``scanForIntersection()``

This function continually runs while the robot is moving forward. So, the robot is moving forward and scanning for an intersection (calling this function) continually one after another. 
If the SONAR servo is currently not moving (etc. 'holding') and it is facing the forward direction then it will know it has 
reached some intersection if the distance in front of it becomes less than a specified distance (the STOP_DISTANCE).
If the SONAR servo is currently not moving and it is facing either left or right, then it will know it has
reached some intersection if the distance to that side is greated than a specified distance (the SCAN_OPEN_DISTANCE).
This function will return true if any kind of intersection is detected.


``scanOpenings(headingSkip)``

Once an intersection has been identified, the robot then needs to check which directions of this intersection are open (etc. can be travelled down) and which are closed (etc. walled).
The headingSkip argument saves the robot a bit of time. This works because the robot has already identified it was at an intersection.
This means we can skip scanning that direction as we already know if it was open or closed.
We can also skip scanning behind us, as we know that this direction must have been open (as we came from this direction).
This leaves two remaining sides to be scanned. Using the algorithm in the function, the SONAR will turn to the remaining two
directions and check whether or not they are open paths. 
This function then records all of the open paths in a list (called openPaths[]).
A waypoint object is then created for this intersection which is passed the list of openPaths[] to 'save' this to that waypoint.
The waypoint is also given information about which direction we came from, or the 'recall' path..
It also adds the path that we came from as being an 'explored path' to the waypoint.


``exploreOpenings()``

The robot has now identified that it is an intersection, stopped and then turned its SONAR to check which directions are valid paths. 
But it now has to choose which path to go down or 'explore'.
To chose this path, we get the current waypoint (information about the intersection we are currently at) and find an open direction of this intersection that we have not explored.
Note that this algorithm loop checks from position 1 of the list rather than position 0. This is because we have obviously 
already explored the path 180 degrees of us (the path we came from).
In the event that no paths can be chosen to explore (as all open paths have already been explored), then the function recallPathway() is called.


``recallPathway()``

This function will turn the robot in the direction that it came from for that current waypoint and then move it forward. This is done in a loop until it finally reaches a waypoint that finally has an 'open path' that is not explored. 
The dead-end waypoints that the robot recalls from are popped from the waypoint list. This is okay as the non-redundunt waypoints that remain will still mark the direction to those dead-end paths as being explored still. 
Ultimately this leads to the robot taking the optimized path of what it has explored during the rescue phase. 



