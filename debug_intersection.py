#!/usr/bin/env python3
'''
This to test how accurate sensors are
or what kinds of values they return
Used for debugging
'''

#library for using fuctions defined by ev3dev etc.
from ev3dev.ev3 import *
from time import sleep

#Important declarations and assignments
button = Button();
gyro = GyroSensor('in1');
gyro.mode = 'GYRO-ANG';
headedDirection = gyro.value();
sonic = UltrasonicSensor('in2');
leftMotor = LargeMotor('outB');
rightMotor = LargeMotor('outC');
smallMotor = MediumMotor('outA');
sonar_state = 1;

FORWARD_LENIANCY = 5;
FORWARD_SPEED = 60;
TURN_SPEED = 800;
STOP_DISTANCE = 230;
SCAN_OPEN_DISTANCE = 400;

def checkManualExit():
    if (button.check_buttons(buttons=['left','right'])):
        exit();

def stopMotors():
    leftMotor.stop();
    rightMotor.stop();

#Keeps the robot moving in the direction it was facing since last turn
def moveForward(speed):
    #if (sonic.value() < STOP_DISTANCE + 140) and (smallMotor.position_sp == 0):            
        #speed = speed/2;        
        #resetSonar();
        #hold the sonar here
    if (gyro.value() - headedDirection > FORWARD_LENIANCY):
        leftMotor.run_direct(duty_cycle_sp=speed * 0.9);
    elif (gyro.value() - headedDirection < -FORWARD_LENIANCY):
        rightMotor.run_direct(duty_cycle_sp=speed * 0.9);
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
    print("Went from: " +str(headedDirection - degree)+" to "+str(headedDirection));

def scanForIntersection():
    global sonar_state;
    if ('holding' in smallMotor.state) and ('stalling' not in smallMotor.state):
        if ((smallMotor.position_sp == 0) and (sonic.value() <= STOP_DISTANCE)):
            return True;    
        elif ((smallMotor.position_sp != 0) and (sonic.value() > SCAN_OPEN_DISTANCE)):
            return True;
        if (smallMotor.position_sp < 0):
            sonar_state = 1;
        if (smallMotor.position_sp > 0):
            sonar_state = -1;   
        smallMotor.run_to_abs_pos(position_sp=smallMotor.position_sp+90*sonar_state, speed_sp=SONAR_SPEED, stop_action="hold");
    return False;

#Main Below Here
smallMotor.run_to_abs_pos(position_sp=0, speed_sp=900, stop_action="hold");
while True:
    checkManualExit();
    moveForward(FORWARD_SPEED);
    if (scanForIntersection()):
        print("Found an intersection!");
        stopMotors();
        num = input(int("Which direction to explore? "));
        turnRelative(num);

