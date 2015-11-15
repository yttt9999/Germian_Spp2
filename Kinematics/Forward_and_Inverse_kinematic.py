__author__ = 'germain'

'''The code is developed to control the stepper motor for SPP2 project IK testing'''
#Import severl libraries
from ctypes import *
import sys
from time import sleep
#Phidget specific imports
from Phidgets.PhidgetException import PhidgetErrorCodes, PhidgetException
from Phidgets.Events.Events import AttachEventArgs, DetachEventArgs, ErrorEventArgs, InputChangeEventArgs, CurrentChangeEventArgs, StepperPositionChangeEventArgs, VelocityChangeEventArgs
from Phidgets.Devices.Stepper import Stepper
from Phidgets.Phidget import PhidgetLogLevel
from kinematics import *
from stepper_motor_setup import *
import math

#Define several important factors about the two stepper motors
STEPPER_1_CIRCLE = 16457
STEPPER_2_CIRCLE = 85970
h2 = 1550.0
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

# def StepperCurrentChanged(e):
#     source = e.device
#     print("Stepper %i: Motor %i -- Current Draw: %6f" % (source.getSerialNum(), e.index, e.current))
#
# def StepperInputChanged(e):
#     source = e.device
#     print("Stepper %i: Input %i -- State: %s" % (source.getSerialNum(), e.index, e.state))
#
# def StepperPositionChanged(e):
#     source = e.device
#     print("Stepper %i: Motor %i -- Position: %f" % (source.getSerialNum(), e.index, e.position))
#
# def StepperVelocityChanged(e):
#     source = e.device
#     print("Stepper %i: Motor %i -- Velocity: %f" % (source.getSerialNum(), e.index, e.velocity))

#Control the stepper motors by converting angels to steps
def step2angel(step,motor_index):
    if motor_index == 1:
        fraction = float(step)/STEPPER_1_CIRCLE
        degree = 360*fraction
        return degree
    else:
        fraction = float(step)/STEPPER_2_CIRCLE
        degree = 360*fraction
        return degree
def angel2step(angel,motor_index):
        if motor_index == 1:
            step = int(angel/0.35*16)
            return step
        else:
            step = int(angel/0.067*16)
            return step
#Main Program Code
try:
	#logging example, uncomment to generate a log file
    #stepper.enableLogging(PhidgetLogLevel.PHIDGET_LOG_VERBOSE, "phidgetlog.log")
    stepper.setOnAttachHandler(StepperAttached)
    stepper.setOnDetachHandler(StepperDetached)
    stepper.setOnErrorhandler(StepperError)
    #stepper.setOnCurrentChangeHandler(StepperCurrentChanged)yt
    # stepper.setOnInputChangeHandler(StepperInputChanged)
    # stepper.setOnPositionChangeHandler(StepperPositionChanged)
    # stepper.setOnVelocityChangeHandler(StepperVelocityChanged)

    #do the samething for stepper_1
    stepper_1.setOnAttachHandler(StepperAttached)
    stepper_1.setOnDetachHandler(StepperDetached)
    stepper_1.setOnErrorhandler(StepperError)
    #stepper_1.setOnCurrentChangeHandler(StepperCurrentChanged)
    # stepper_1.setOnInputChangeHandler(StepperInputChanged)
    # stepper_1.setOnPositionChangeHandler(StepperPositionChanged)
    # stepper_1.setOnVelocityChangeHandler(StepperVelocityChanged)
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
    setup_limit(stepper,0,1645*10,1.5,1645*10)#speed normally 4000
    setup_limit(stepper_1,0,8597*10,1.0,8597*10)
    sleep(2)
    try:
        #report where they are
        print ("the current position for motor 1 is %lf"% (step2angel(stepper.getCurrentPosition(0),1)))
        print ("the current position for motor 2 is %lf"% (step2angel(stepper_1.getCurrentPosition(0),2)))
        while(True):
            model = raw_input("Please choose model, 1 for FK, 2 for IK, 3 for back to initial position, 4 for exit")
            if model == "1":
                target_degree_theta1= float(raw_input("Please enter the degree of bottom motor:"))
                target_degree_theta2= float(raw_input("Please enter the degree of upper motor:"))
                if target_degree_theta2 <= 0:
                    print "Warnning! Cannot caululate FK"
                elif target_degree_theta1 > 0:
                    target_position_stepper = angel2step(target_degree_theta1,1)
                    target_position_stepper_1 = angel2step(target_degree_theta2,2)
                    stepper.setTargetPosition(0,target_position_stepper)
                    stepper_1.setTargetPosition(0,-1*target_position_stepper_1)
                    target_degree_theta1 = math.radians(float(target_degree_theta1))
                    target_degree_theta2 = math.radians(float(target_degree_theta2))
                    b = (h2 + r2*math.cos(target_degree_theta2))/math.tan(target_degree_theta2)+r2*math.sin(target_degree_theta2)+r1/math.tan(target_degree_theta1)
                    FK_X = r1/math.cos(target_degree_theta1)-b*math.sin(target_degree_theta1)
                    FK_Y = b*math.cos(target_degree_theta1)
                    print("real_world_codinate: (%lf,%lf)"%(FK_X,FK_Y))
                elif target_degree_theta1 < 0:
                    target_position_stepper = angel2step(target_degree_theta1,1)
                    target_position_stepper_1 = angel2step(target_degree_theta2,2)
                    stepper.setTargetPosition(0,target_position_stepper)
                    stepper_1.setTargetPosition(0,-1*target_position_stepper_1)
                    target_degree_theta1 = math.radians(float(target_degree_theta1))
                    target_degree_theta2 = math.radians(float(target_degree_theta2))
                    b = (h2 + r2*math.cos(target_degree_theta2))/math.tan(target_degree_theta2)+r2*math.sin(target_degree_theta2)+r1/math.tan(-1*target_degree_theta1)
                    FK_X = b*math.sin(-1.0*target_degree_theta1)
                    FK_Y = b*math.cos(-1.0*target_degree_theta1)-r1/math.sin(-1.0*target_degree_theta1)
                    print("real_world_codinate: (%lf,%lf)"%(FK_X,FK_Y))
                elif target_degree_theta1 == 0:
                    target_position_stepper = angel2step(target_degree_theta1,1)
                    target_position_stepper_1 = angel2step(target_degree_theta2,2)
                    stepper.setTargetPosition(0,target_position_stepper)
                    stepper_1.setTargetPosition(0,-1*target_position_stepper_1)
                    target_degree_theta1 = math.radians(float(target_degree_theta1))
                    target_degree_theta2 = math.radians(float(target_degree_theta2))
                    FK_X = r1
                    FK_Y = (h2 + r2*math.cos(target_degree_theta2))/math.tan(target_degree_theta2)+r2*math.sin(target_degree_theta2)
                    print("real_world_codinate: (%lf,%lf)"%(FK_X,FK_Y))
                else:
                    pass
            elif model == "2":
                target_real_world_x = float(raw_input("Please enter the real world X:"))
                target_real_world_y = float(raw_input("Please enter the real world Y:"))
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
                stepper.setTargetPosition(0,target_position_stepper)
                stepper_1.setTargetPosition(0,-1*target_position_stepper_1)
            elif model == "3":
                stepper.setTargetPosition(0,0)
                stepper_1.setTargetPosition(0,0)
            elif model == "4":
                break
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
