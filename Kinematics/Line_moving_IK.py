__author__ = 'germain'

'''The code is developed to control the stepper motor for SPP2 project IK testing'''
#Import severl libraries
from ctypes import *
import sys
from time import sleep
#Phidget specific imports
from Phidgets.PhidgetException import PhidgetErrorCodes, PhidgetException
from Phidgets.Devices.Stepper import Stepper
from kinematics import *
from stepper_motor_setup import *
import math
import numpy as np

#Define several important factors about the two stepper motors
STEPPER_1_CIRCLE = 16457
STEPPER_2_CIRCLE = 85970
h2 = 1518.0
r1 = 30.0
r2 = 35.0
try:
    stepper = Stepper()
    #Create a new device object
    stepper_1 = Stepper()
except RuntimeError as e:
    print("Runtime Exception:%s"% e.details)
    print("Exiting....")
    exit(1)

#Information Display Function
def DisplayDeviceInfo():
    print("|------------|----------------------------------|--------------|------------|")
    print("|- Attached -|-              Type              -|- Serial No. -|-  Version -|")
    print("|------------|----------------------------------|--------------|------------|")
    print("|- %8s -|- %30s -|- %10d -|- %8d -|" % (stepper.isAttached(), stepper.getDeviceName(), stepper.getSerialNum(), stepper.getDeviceVersion()))
    print("|------------|----------------------------------|--------------|------------|")
    print("Number of Motors: %i" % (stepper.getMotorCount()))
    print("bottom motor Acc max= %lf, Acc min=%lf" %(stepper.getAccelerationMax(0),stepper.getAccelerationMin(0)))
    print("upper motor Acc max= %lf, Acc min=%lf" %(stepper_1.getAccelerationMax(0),stepper_1.getAccelerationMin(0)))
    print("bottom motor V max= %lf,V min=%lf" %(stepper.getVelocityMax(0),stepper.getVelocityMin(0)))
    print("upper motor V max= %lf,V min=%lf" %(stepper_1.getVelocityMax(0),stepper_1.getVelocityMin(0)))

#Event Handler Callback Functions
def StepperAttached(e):
    attached = e.device
    print("Stepper %i Attached!" % (attached.getSerialNum()))

def StepperDetached(e):
    detached = e.device
    print("Stepper %i Detached!" % (detached.getSerialNum()))

def StepperError(e):
    try:
        source = e.device
        print("Stepper %i: Phidget Error %i: %s" % (source.getSerialNum(), e.eCode, e.description))
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))

def step2angel(step,motor_index):
    if motor_index == 1:
        fraction = float(step)/STEPPER_1_CIRCLE
        degree = 360*fraction
        return degree
    else:
        fraction = float(step)/STEPPER_2_CIRCLE
        degree = 360*fraction
        return degree
# def angel2step(angel,motor_index):
#         if motor_index == 1:
#             step = int(angel/0.35*16)
#             return step
#         else:
#             step = int(angel/0.067*16)
#             return step
def angel2step(angel,motor_index):
        if motor_index == 1:
            step = int(angel*STEPPER_1_CIRCLE/360)
            return step
        else:
            step = int(angel*STEPPER_2_CIRCLE/360)
            return step
def Inverse_Kinematics(target_real_world_x,target_real_world_y):
    if target_real_world_y < 0:
        print "not reachable"
        target_degree_theta1 = 0
        target_degree_theta2 = 0
    elif target_real_world_x > r1:
        target_degree_theta1 = -1.0*math.degrees(math.acos(r1/math.sqrt(target_real_world_x**2+target_real_world_y**2))-math.atan(abs(target_real_world_y)/abs(target_real_world_x)))
        target_degree_theta2 = (90.0-math.degrees(math.pi/2.0-math.atan(h2/math.sqrt(target_real_world_x**2+target_real_world_y**2-r1**2))-math.atan(r2/math.sqrt(target_real_world_x**2+target_real_world_y**2-r1**2+h2**2))))
    else:
        target_degree_theta1 = math.degrees(math.pi-math.acos(r1/math.sqrt(target_real_world_x**2+target_real_world_y**2))-math.atan(abs(target_real_world_y)/abs(target_real_world_x)))
        target_degree_theta2 = math.degrees(math.pi-math.atan(math.sqrt(target_real_world_x**2+target_real_world_y**2-r1**2)/h2)-math.acos(r2/math.sqrt(target_real_world_x**2+target_real_world_y**2-r1**2+h2**2)))
    print target_degree_theta1,target_degree_theta2
    target_position_stepper = angel2step(float(target_degree_theta1),1)
    target_position_stepper_1 = angel2step(float(target_degree_theta2),2)
    return [target_position_stepper,target_position_stepper_1]
    # stepper.setTargetPosition(0,target_position_stepper)
    # stepper_1.setTargetPosition(0,-1*target_position_stepper_1)



#Main Program Code
try:
	#logging example, uncomment to generate a log file
    stepper.setOnAttachHandler(StepperAttached)
    stepper.setOnDetachHandler(StepperDetached)
    stepper.setOnErrorhandler(StepperError)


    #do the samething for stepper_1
    stepper_1.setOnAttachHandler(StepperAttached)
    stepper_1.setOnDetachHandler(StepperDetached)
    stepper_1.setOnErrorhandler(StepperError)

except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))
    print("Exiting....")
    exit(1)

print("Opening phidget object....")

try:
    stepper.openPhidget(serial = 398177)
    stepper_1.openPhidget(serial = 398175)
except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))
    print("Exiting....")
    exit(1)

print("Waiting for attach....")

try:
    stepper.waitForAttach(10000)
    stepper_1.waitForAttach(10000)
except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))
    try:
        stepper.closePhidget()
        stepper_1.closePhidget()
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Exiting....")
        exit(1)
    print("Exiting....")
    exit(1)
else:
    DisplayDeviceInfo()

#Here We start our code to move the motor as we expected
try:
    #set up the zero position
    stepper.setCurrentPosition(0,0)
    stepper_1.setCurrentPosition(0,0)
    #Start the stepper motors
    stepper.setEngaged(0,True)
    stepper_1.setEngaged(0,True)
    #Set up the speed, acceleration,and current
    setup_limit(stepper,0,1645*30,1.5,1645*5)#speed normally 4000
    setup_limit(stepper_1,0,8597*30,1.5,8597*5)
    sleep(2)
    try:
        #report where they are
        print ("the current position for motor 1 is %lf"% (step2angel(stepper.getCurrentPosition(0),1)))
        print ("the current position for motor 2 is %lf"% (step2angel(stepper_1.getCurrentPosition(0),2)))
        while(True):
            model = raw_input("Please choose model, 1 for Drawing Line, 2 for back to initial position, 3 for drawing 5-stars, 4 for exit")
            if model == "1":
                Line_start_pos_x = float(raw_input("Please enter the X of start point of the line"))
                Line_start_pos_y = float(raw_input("Please enter the Y of start point of the line"))
                Line_end_pos_x = float(raw_input("Please enter the X of end point of the line"))
                Line_end_pos_y = float(raw_input("Please enter the Y of end point of the line"))
                setup_limit(stepper,0,1645*30,1.5,1645*5)#speed normally 4000
                setup_limit(stepper_1,0,8597*30,1.5,8597*5)
                start_step = Inverse_Kinematics(Line_start_pos_x,Line_start_pos_y)
                stepper.setTargetPosition(0,start_step[0])
                stepper_1.setTargetPosition(0,-1*start_step[1])
                sleep(2)
                end_step = Inverse_Kinematics(Line_end_pos_x,Line_end_pos_y)
                x = np.linspace(start_step[0],end_step[0],30)
                y = np.linspace(start_step[1],end_step[1],30)
                # print x
                # print y
                # stepper_diff = abs(end_step[0]-start_step[0])
                # stepper_1_diff = abs(end_step[1]-start_step[1])
                # stepper_diff_time = stepper_diff / 1645.7
                # stepper_1_diff_time = stepper_1_diff / 8597.0
                setup_limit(stepper,0,int(abs(x[1]-x[0])*10000),1.0,int(abs(x[1]-x[0])*10))
                setup_limit(stepper_1,0,int(abs(y[1]-y[0])*10000),1.0,int(abs(y[1]-y[0])*10))
                # print int(abs(y[1]-y[0])*1000)
                # print int(abs(y[1]-y[0])*10)
                for i in range(len(x)):
                    stepper.setTargetPosition(0,int(x[i]))
                    stepper_1.setTargetPosition(0,-1*int(y[i]))
                    # while (stepper.getCurrentPosition(0) != stepper.getTargetPosition(0)):
                    #     pass
                    # while (stepper_1.getCurrentPosition(0) != stepper_1.getTargetPosition(0)):
                    #     pass


            elif model == "2":
                setup_limit(stepper,0,1645*30,1.5,1645*5)#speed normally 4000
                setup_limit(stepper_1,0,8597*30,1.5,8597*5)
                stepper.setTargetPosition(0,0)
                stepper_1.setTargetPosition(0,0)
            elif model == "4":
                break
            elif model == "3":
                star = [-600,1600,-800,3000,-1000,1600,-400,2600,-1200,2600,-600,1600]
                start_star = Inverse_Kinematics(star[0],star[1])
                stepper.setTargetPosition(0,start_star[0])
                stepper_1.setTargetPosition(0,-1*start_star[1])
                sleep(2)
                for i in range(len(star)/2-1):
                    print ("next position X:%lf Y:%lf"%(star[2*i+2],star[2*i+3]))
                    star_mid_point = Inverse_Kinematics(star[2*i+2],star[2*i+3])
                    star_mid_point_bottom_motor = np.linspace(start_star[0],star_mid_point[0],50)
                    star_mid_point_upper_motor = np.linspace(start_star[1],star_mid_point[1],50)
                    setup_limit(stepper,0,int(abs(star_mid_point_bottom_motor[1]-star_mid_point_bottom_motor[0])*10000),1.0,int(abs(star_mid_point_bottom_motor[1]-star_mid_point_bottom_motor[0])*500))
                    setup_limit(stepper_1,0,int(abs(star_mid_point_upper_motor[1]-star_mid_point_upper_motor[0])*10000),1.0,int(abs(star_mid_point_upper_motor[1]-star_mid_point_upper_motor[0])*500))
                    for k in range(len(star_mid_point_bottom_motor)):
                         stepper.setTargetPosition(0,int(star_mid_point_bottom_motor[k]))
                         stepper_1.setTargetPosition(0,-1*int(star_mid_point_upper_motor[k]))
                    start_star = star_mid_point
                    while (stepper.getCurrentPosition(0) != stepper.getTargetPosition(0)):
                        pass
                    while (stepper_1.getCurrentPosition(0) != stepper_1.getTargetPosition(0)):
                        pass
            else:
                print "ERROR INPUT"

            while (stepper.getCurrentPosition(0) != stepper.getTargetPosition(0)):
                pass
            while (stepper_1.getCurrentPosition(0) != stepper_1.getTargetPosition(0)):
                pass
            sleep(1)
            # print ("the current position for motor 1 is %lf"% (stepper.getCurrentPosition(0)))
            # print ("the current position for motor 2 is %lf"% (stepper_1.getCurrentPosition(0)))
    except KeyboardInterrupt:
        setup_limit(stepper,0,1645*30,1.5,1645*5)#speed normally 4000
        setup_limit(stepper_1,0,8597*30,1.5,8597*5)
        stepper.setTargetPosition(0,0)
        stepper_1.setTargetPosition(0,0)
        while (stepper.getCurrentPosition(0) != stepper.getTargetPosition(0)):
            pass
        while (stepper_1.getCurrentPosition(0) != stepper_1.getTargetPosition(0)):
            pass
    print("down")
    stepper.setEngaged(0,False)
    stepper_1.setEngaged(0,False)
    sleep(1)
    stepper.closePhidget()
    stepper_1.closePhidget()

except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))
    print("Exiting....")
    exit(1)

exit(0)
