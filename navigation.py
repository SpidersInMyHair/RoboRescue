#!/usr/bin/env python3

''' 
    ### CHANGELIST ###
 Added the color sensor checking for orange (only does this when robot is moving forward). If it detects orange, motors should stop, robot should beep and RETURN_PHASE set to True.
 Added loop backtracking only if necessary. So does not backtrack a loop if every waypoint in that loop is explored.
 Added forward alignment as well as reverse alignment when a forward closed wall is available to use as reference for alignment.
'''

# library for using fuctions defined by ev3dev etc.
from ev3dev.ev3 import *
from time import sleep

# Important declarations and assignments.
button = Button();
display = Screen();
gyro = GyroSensor();
sonic = UltrasonicSensor('in3');
l_sonic = UltrasonicSensor('in4');
color = ColorSensor();
leftMotor = LargeMotor('outA');
clawMotor = LargeMotor('outB')
smallMotor = MediumMotor('outC');
rightMotor = LargeMotor('outD');
gyro.mode = 'GYRO-RATE'
gyro.mode = 'GYRO-ANG';
color.mode = 'COL-COLOR';
headedDirection = gyro.value();
global g_x_pos;
global g_y_pos;
g_x_pos = 0; 
g_y_pos = 0;

# Important 'Constants' These should BE CHANGED BASED ON PHYSICAL DESIGN OF THE ROBOT.
FORWARD_SPEED = 60;
FORWARD_LENIANCY = 1;
DISTANCE_UNIT = 896;
STOP_DISTANCE = 175;
SCAN_OPEN_DISTANCE = 360;
FORWARD_CLEARANCE = 180;
TURN_SPEED = 300;
SONAR_SPEED = 800;
SONAR_RESET_SLEEP = 1250;       
RETURN_PHASE = False;

# Prints this so we know program is not stuck building.
print("Building Complete");

# A Waypoint is an object that is made at every new intersection.
# It records which directions are open, which directions have been explored and which direction the robot came from as well as the coordinates.
class Waypoint(object):
    def __init__(self, openPaths, x_pos, y_pos):
        self.openPaths = openPaths;
        self.exploredPaths = [];
        self.x_pos = x_pos;
        self.y_pos = y_pos;
    
    def getOpenPaths(self):
        return self.openPaths;

    def getRecallPath(self):
        return self.openPaths[0];

    def addExploredPath(self, heading):
        self.exploredPaths.append(heading);

    def getExploredPaths(self):
        return self.exploredPaths;

    def isDirectionExplored(self, heading):
        if (heading in self.exploredPaths):
            return True;
        return False; 

    def hasUnexplored(self):
        if (len(self.openPaths) > len(self.exploredPaths)):
            return True;
        return False;
        
    def get_x_pos(self):
        return self.x_pos;
        
    def get_y_pos(self):
        return self.y_pos;

    def printOpenings(self):
        print("\t ", self.openPaths, end='');

    def printExplored(self):
        print("\t", self.exploredPaths, end='');

my_waypoints = [];


# Debug function used to print important information.
def debug():
    print("Scan at intersection #" + str(len(my_waypoints)));
    print("This intersection has coords: " + str(my_waypoints[len(my_waypoints) - 1].get_x_pos()) + ", " + str(my_waypoints[len(my_waypoints) - 1].get_y_pos()));
    print(" OpenPaths: ");
    my_waypoints[len(my_waypoints) - 1].printOpenings();
    print("");
    print(" ExploredPaths: ");
    my_waypoints[len(my_waypoints) - 1].printExplored();
    print("");

    
# This function is the condition for the main loop to run. Returns False if specified buttons are pressed.
def checkManualExit():
    if (button.check_buttons(buttons=['backspace'])):
        stopMotors();
        exit();
    return True;


# Run once at program start, before entering into the main loop.
def initialization():
  	smallMotor.run_to_abs_pos(position_sp=0, speed_sp=500, stop_action="hold");
    resetSonar();
    leftMotor.polarity = 'inversed';
    rightMotor.polarity = 'inversed';
    clawMotor.run_to_rel_pos(position_sp = 0, speed_sp = 10*FORWARD_SPEED, stop_action = "hold");
    
    
# Returns the heading given as an argument that has been approximated to the nearest -90, 0, 90 or 180 angle.
def getGlobalDirection(heading):
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

  
# Immediately stop both the drive motors.
def stopMotors():
    leftMotor.stop();
    rightMotor.stop();


# Turn the turret ultrasonic sensor to left, forward or right. These values are respectively represented by -1, 0, 1.
def turnSonar(position):
    if (smallMotor.position_sp != 90*position):
        smallMotor.run_to_abs_pos(position_sp = 90*position, speed_sp = SONAR_SPEED, stop_action = "hold");
        smallMotor.wait_while('running');
        sleep(2.0);                         #Hold the sonar here for at least 2.0 seconds so that a true value can be obtained


# Turn the turrent ultrasonic sensor to the forward position. Hold it here.
def resetSonar():
    turnSonar(0);
    smallMotor.run_timed(time_sp = SONAR_RESET_SLEEP, speed_sp = 0, stop_action = "hold");

    
# Simply runs the claw motor sufficiently to put the claw into a 'closed' position
def closeClaw():
    clawMotor.run_to_rel_pos(position_sp = -70, speed_sp = 10*FORWARD_SPEED, stop_action = "hold");
    sleep(1.0);

    
# Move the robot forward a set amount of motor rotations.
# Whilst moving forward, the color sensor is also checking for the color orange.
def moveForwardSet(amount, change_coords):
    global g_x_pos;
    global g_y_pos;
    global RETURN_PHASE;
    leftMotor.run_to_rel_pos(position_sp = amount, speed_sp = 10*FORWARD_SPEED, stop_action = "hold");
    rightMotor.run_to_rel_pos(position_sp = amount, speed_sp = 10*FORWARD_SPEED, stop_action = "hold");
    while ('running' in rightMotor.state):
        if (gyro.value() - headedDirection > FORWARD_LENIANCY):
            leftMotor.speed_sp = FORWARD_SPEED * 0.9;
            rightMotor.speed_sp = FORWARD_SPEED;
        elif (gyro.value() - headedDirection < -FORWARD_LENIANCY):
            leftMotor.speed_sp = FORWARD_SPEED;
            rightMotor.speed_sp = FORWARD_SPEED * 0.9;
        else:
            leftMotor.speed_sp = FORWARD_SPEED;
            rightMotor.speed_sp = FORWARD_SPEED;
        if (color.value() == 5 and RETURN_PHASE == False):
            stopMotors();
            Sound.beep();
            closeClaw();
            RETURN_PHASE = True;
            
    if (change_coords == True):
        g_x_pos += (getGlobalDirection(gyro.value()) == 90) - (getGlobalDirection(gyro.value()) == -90);
        g_y_pos += (getGlobalDirection(gyro.value()) == 0) - (getGlobalDirection(gyro.value()) == 180);

        
# Need to test this function and CHANGE THE VALUES SO THAT IT WORKS NICELY!!!
def alignment_reverse():
    # Forward_clearance is currently set to 150. So robot will aim to align within 15cm to 20cm space ahead of sonar.
    while (sonic.value() < FORWARD_CLEARANCE):
        moveForwardSet(-5, False); 
    while (sonic.value() > FORWARD_CLEARANCE + 40):
        moveForwardSet(5, False);

        
# A positive degree will turn this clockwise and a negative degree will turn anti-clockwise.
def turnRelative(degree):
    global headedDirection     
    goalHeader = headedDirection + degree;
    while (goalHeader != gyro.value()):
        remainingTurn = goalHeader - gyro.value()
        speedModifier = (remainingTurn)/90.0;
        if (TURN_SPEED*speedModifier < -1000):
            speedModifier = -1000/TURN_SPEED;
        if (TURN_SPEED*speedModifier > 1000):
            speedModifier = 1000/TURN_SPEED;
        leftMotor.run_forever(speed_sp = TURN_SPEED*speedModifier);
        rightMotor.run_forever(speed_sp = -TURN_SPEED*speedModifier); 
    stopMotors(); 
    resetSonar();
    headedDirection = getGlobalDirection(gyro.value());


# Once at an intersection, looks around at an intersection to record where any openings are.
def scanOpenings(global_x_pos, global_y_pos):
    # Ensure alignment before manoveres
    realDirection = gyro.value() % 360;
    if (realDirection >= 225):
        realDirection = realDirection - 360;
    turnRelative(getGlobalDirection(headedDirection - gyro.value()));  
    
    openPaths = [];
    openPaths.append(getGlobalDirection(gyro.value() - 180));
    
    if (l_sonic.value() * 10 > SCAN_OPEN_DISTANCE):
        openPaths.append(getGlobalDirection(gyro.value() - 90));
    if (sonic.value() > SCAN_OPEN_DISTANCE):
        openPaths.append(getGlobalDirection(gyro.value()));
    else:
        alignment_reverse();

    turnSonar(1);
    if (sonic.value() > SCAN_OPEN_DISTANCE):
        openPaths.append(getGlobalDirection(gyro.value() + 90));
            
    my_waypoints.append(Waypoint(openPaths, global_x_pos, global_y_pos));
    my_waypoints[len(my_waypoints) - 1].addExploredPath(getGlobalDirection(gyro.value() - 180));
    debug(); 
    checkLoop();


# Every time a new waypoint is created, this function should be called to check if the robot is in a maze-loop. If so, it will seal of the loop path by marking either entrance as explored.
# If the loop contains other unexplored paths then backtack through the loop. If the loop contains no unexplored paths, simply delete the loop waypoints and seal the loop off.
def checkLoop():
    for i in range(0, len(my_waypoints) - 1):  
        if (my_waypoints[i].get_x_pos() == my_waypoints[len(my_waypoints) - 1].get_x_pos()) and (my_waypoints[i].get_y_pos() == my_waypoints[len(my_waypoints) - 1].get_y_pos()):
            # Not a true new intersection.
            Sound.beep();
            my_waypoints[i].addExploredPath(getGlobalDirection(my_waypoints[len(my_waypoints) - 1].getRecallPath()));
            backtrack = False;
            for j in range (i, len(my_waypoints) - 1):
                if (my_waypoints[j].hasUnexplored() == True):
                    backtrack = True;
            if (backtrack == True):
                turnRelative(getGlobalDirection(my_waypoints[len(my_waypoints) - 1].getRecallPath() - gyro.value()));
                moveForwardSet(DISTANCE_UNIT, True);
                turnRelative(getGlobalDirection(headedDirection - gyro.value()));
                my_waypoints.pop(); # Destroy the repeated/looped intersection   
            else:
                for j in range (i + 1, len(my_waypoints) - 1):
                    del my_waypoints[i+1];
                my_waypoints.pop();
            break;


# For choosing and going down a NEW open pathway of intersection, not used on FINAL rescue phase. This function simply turns the robot in the direction of an unexplored path.
def exploreOpenings():  
    if (RETURN_PHASE == False):
        choices = my_waypoints[len(my_waypoints) - 1].getOpenPaths();   
        for i in reversed(range(1, len(choices))):                      
            if (my_waypoints[len(my_waypoints) - 1].isDirectionExplored(choices[i]) == False):
                turnRelative(getGlobalDirection(choices[i] - gyro.value())); 
                my_waypoints[len(my_waypoints) - 1].addExploredPath(getGlobalDirection(gyro.value()));
                return;
    recallPathway();
   

# Begin following recall path until waypoint with unexplored path found or forever if in rescue phase.
def recallPathway():
    while (my_waypoints[len(my_waypoints) - 1].hasUnexplored() == False) or (RETURN_PHASE == True):
        turnRelative(getGlobalDirection(my_waypoints[len(my_waypoints) - 1].getRecallPath() - gyro.value()));
        moveForwardSet(DISTANCE_UNIT, True);  
        turnRelative(getGlobalDirection(headedDirection - gyro.value()));
        sleep(1.0);
        my_waypoints.pop();
    exploreOpenings();      

    
#Main Below Here
initialization();
while checkManualExit():
    moveForwardSet(DISTANCE_UNIT, True);
    scanOpenings(g_x_pos, g_y_pos);
    exploreOpenings(); 
