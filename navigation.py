#!/usr/bin/env python3
''' 
See README for more information
''''''
#NEED TO ADD: Try to cut out sleep times
#NEED TO ADD: Fine-Tuned turns on recall path
'''

#library for using fuctions defined by ev3dev etc.
from ev3dev.ev3 import *
from time import sleep

#Important declarations and assignments
button = Button();
display = Screen();
gyro = GyroSensor('in1');
sonic = UltrasonicSensor('in2');
smallMotor = MediumMotor('outA');
leftMotor = LargeMotor('outB');
rightMotor = LargeMotor('outC');
gyro.mode = 'GYRO-ANG';
headedDirection = gyro.value();
smallMotor.run_to_abs_pos(position_sp=0, speed_sp=500, stop_action="hold");
sonar_state = 1;

#Important 'Constants' These should BE CHANGED BASED ON PHYSICAL DESIGN OF THE ROBOT
FORWARD_SPEED = 60;
FORWARD_LENIANCY = 5;
STOP_DISTANCE = 220;
SCAN_OPEN_DISTANCE = 400;
TURN_SPEED = 800;
SONAR_SPEED = 800;
SONAR_RESET_SLEEP = 1150;       
RETURN_PHASE = False;

#Prints this so we know program is not stuck building
print("Initialization Complete");

#A Waypoint is an object that is made at every NEW intersection
#It records which directions are open, which directions have been explored
#and which direction the robot came from
class Waypoint(object):
    def __init__(self, openPaths):
        self.openPaths = openPaths;
        self.exploredPaths = [];
    
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

    def printOpenings(self):
        print("\t ", self.openPaths, end='');

    def printExplored(self):
        print("\t", self.exploredPaths, end='');

my_waypoints = [];

def debug():
    print("Scan: ");
    print(" OpenPaths: ");
    for i in range(0, len(my_waypoints)):
        my_waypoints[i].printOpenings();
    print("");
    print(" ExploredPaths: ");
    for i in range(0, len(my_waypoints)):
        my_waypoints[i].printExplored();
    print("");

def checkManualExit():
    if (button.check_buttons(buttons=['left','right'])):
        stopMotors();
        exit();
    return True;

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

def stopMotors():
    leftMotor.stop();
    rightMotor.stop();

def turnSonar(position):
    if (smallMotor.position_sp != 90*position):
        smallMotor.run_to_abs_pos(position_sp=90*position, speed_sp=SONAR_SPEED, stop_action="hold");
        smallMotor.wait_while('running');
        sleep(0.5);

def resetSonar():
    turnSonar(0);
    smallMotor.run_timed(time_sp=SONAR_RESET_SLEEP, speed_sp=0, stop_action="hold");
    

#Keeps the robot moving in the direction it was facing since last turn
def moveForward(speed):
    if (gyro.value() - headedDirection > FORWARD_LENIANCY):
        leftMotor.run_direct(duty_cycle_sp=speed * 0.9);
        rightMotor.run_direct(duty_cycle_sp=speed);
    elif (gyro.value() - headedDirection < -FORWARD_LENIANCY):
        rightMotor.run_direct(duty_cycle_sp=speed * 0.9);
        leftMotor.run_direct(duty_cycle_sp=speed);
    else:
        leftMotor.run_direct(duty_cycle_sp=speed);
        rightMotor.run_direct(duty_cycle_sp=speed);

#A positive degree will turn this clockwise and a negative degree will turn anti-clockwise
def turnRelative(degree):
    global headedDirection     
    goalHeader = headedDirection + degree;
    while (goalHeader != gyro.value()):
        remainingTurn = goalHeader - gyro.value()
        if (abs(remainingTurn) < 45):
            speedModifier = (remainingTurn)/90;
        else:
            speedModifier = 1*(degree/abs(degree));
        leftMotor.run_forever(speed_sp=TURN_SPEED*speedModifier);
        rightMotor.run_forever(speed_sp=-TURN_SPEED*speedModifier); 
    stopMotors(); 
    resetSonar();
    headedDirection = gyro.value();


#Continuously runs while robot is moving forward to check if at an intersection
def scanForIntersection():
    global sonar_state;
    if ('holding' in smallMotor.state):
        if ((smallMotor.position_sp == 0) and (sonic.value() <= STOP_DISTANCE + 150)):
            while (sonic.value() > STOP_DISTANCE):
                pass;
            return True;    
        elif ((smallMotor.position_sp != 0) and (sonic.value() > SCAN_OPEN_DISTANCE)):
            return True;
    if ('holding' in smallMotor.state) and ('stalling' not in smallMotor.state):
        if (smallMotor.position_sp < 0):
            sonar_state = 1;
        if (smallMotor.position_sp > 0):
            sonar_state = -1;   
        smallMotor.run_to_abs_pos(position_sp=smallMotor.position_sp+90*sonar_state, speed_sp=SONAR_SPEED, stop_action="hold");
    return False;


#Once at an intersection, looks around at an intersection to record where any openings are 
def scanOpenings():
    sleep(1.0);   #let the gyro settle before passing a value to turn_Relative
    turnRelative(getGlobalDirection(headedDirection - gyro.value()));           #Ensure alignment before manouveres
    openPaths = [];
    openPaths.append(getGlobalDirection(gyro.value() - 180));
    for i in range (-1,2):
        turnSonar(i);
        sleep(1.0);           #have to let the gyro get a value - otherwise will be a false value
        print("     scanner position (not skipped): " + str(smallMotor.position_sp) + " @ " + str(sonic.value()));
        if (sonic.value() > SCAN_OPEN_DISTANCE):
            openPaths.append(getGlobalDirection(gyro.value() + 90*i));
    my_waypoints.append(Waypoint(openPaths));
    my_waypoints[len(my_waypoints) - 1].addExploredPath(getGlobalDirection(gyro.value() - 180)); 


#For choosing and going down a NEW open pathway of intersection, not used on FINAL rescue phase
def exploreOpenings():  
    debug();
    choices = my_waypoints[len(my_waypoints) - 1].getOpenPaths();   
    for i in reversed(range(1, len(choices))):                      
        if (my_waypoints[len(my_waypoints) - 1].isDirectionExplored(choices[i]) == False):
            turnRelative(getGlobalDirection(choices[i] - gyro.value())); 
            my_waypoints[len(my_waypoints) - 1].addExploredPath(getGlobalDirection(gyro.value()));
            return;
    recallPathway();
   

#Begin following recall path until waypoint with unexplored path found
def recallPathway():
    while (my_waypoints[len(my_waypoints) - 1].hasUnexplored() == False):
        sleep(1);
        turnRelative(getGlobalDirection(my_waypoints[len(my_waypoints) - 1].getRecallPath()-gyro.value()));
        while (not scanForIntersection()): 
            moveForward(FORWARD_SPEED);
        stopMotors();
        turnRelative(getGlobalDirection(headedDirection - gyro.value()));
        my_waypoints.pop();
    exploreOpenings();      


#Main Below Here
resetSonar();
while checkManualExit():
    moveForward(FORWARD_SPEED);
    if (scanForIntersection()): 
        stopMotors();
        scanOpenings();
        exploreOpenings(); 
