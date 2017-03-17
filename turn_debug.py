#!/usr/bin/env python3
''' 
Spins the robot for the given angle
Used for debugging and testing physical constraints
'''

#library for using fuctions defined by ev3dev etc.
from ev3dev.ev3 import *
from time import sleep

#Important declarations and assignments
button = Button();
display = Screen();
gyro = GyroSensor();
gyro.mode = 'GYRO-ANG';
headedDirection = gyro.value();
leftMotor = LargeMotor('outB');
rightMotor = LargeMotor('outC');

#Important 'Constants'
TURN_SPEED = 500;

#Prints this so we know program is not stuck building
print("Initialization Complete");

def stopMotors():
	leftMotor.stop();
	rightMotor.stop();

def getGlobalDirection(heading):
    reducedHeading = (heading % 360);
    if (reducedHeading >= 315 or reducedHeading < 45):
        return 0;
    if (reducedHeading >= 45 and reducedHeading < 135):
        return 90;
    if (reducedHeading >= 135 and reducedHeading < 225):
        return 180;
    if (reducedHeading >= 225 and reducedHeading < 315):
        return -90; 

#A positive degree will turn this clockwise and a negative degree will turn anti-clockwise
#Turns independent of friction, wheel rotations etc.by using the gyroscope. Very precise but slower.
def turnRelative(degree):
    global headedDirection     
    goalHeader = headedDirection + degree;
    while (goalHeader != gyro.value()):
        remainingTurn = goalHeader - gyro.value()
        if (abs(remainingTurn) < 45):
            speedModifier = (remainingTurn)/90;
        else:
            speedModifier = 1*(degree/abs(degree));
        leftMotor.run_forever(speed_sp=TURN_SPEED*speedModifier, stop_action="hold");
        rightMotor.run_forever(speed_sp=-TURN_SPEED*speedModifier, stop_action="hold"); 
    stopMotors(); 
    resetSonar();
    headedDirection = gyro.value();
    print("Went from: " +str(headedDirection - degree)+" to "+str(headedDirection)

#Main Below Here
while (True):
    num = int(input("How much to rotate? "));
    turnRelative(getGlobalDirection(num));
