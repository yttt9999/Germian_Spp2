__author__ = 'kevin'
'''This file defines all the functions that is needed to control the stepper motor'''
#Phidget specific imports
from Phidgets.PhidgetException import PhidgetErrorCodes, PhidgetException
from Phidgets.Devices.Stepper import Stepper

#Define several important factors about the two stepper motors
STEPPER_77_CIRCLE = 16457
STEPPER_75_CIRCLE = 85970
STEPPER_77_STEP = 0.35
STEPPER_75_STEP = 0.067
Phidget_77 = 398177
Phidget_75 = 398175
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

def StepperCurrentChanged(e):
    source = e.device
    print("Stepper %i: Motor %i -- Current Draw: %6f" % (source.getSerialNum(), e.index, e.current))

def StepperInputChanged(e):
    source = e.device
    print("Stepper %i: Input %i -- State: %s" % (source.getSerialNum(), e.index, e.state))

def StepperPositionChanged(e):
    source = e.device
    print("Stepper %i: Motor %i -- Position: %f" % (source.getSerialNum(), e.index, e.position))

def StepperVelocityChanged(e):
    source = e.device
    print("Stepper %i: Motor %i -- Velocity: %f" % (source.getSerialNum(), e.index, e.velocity))

#define functions to set up the stepper motors
def setup_handeller(stepper):
    stepper.setOnAttachHandler(StepperAttached)
    stepper.setOnDetachHandler(StepperDetached)
    stepper.setOnErrorhandler(StepperError)
    #stepper.setOnCurrentChangeHandler(StepperCurrentChanged)
    #stepper.setOnInputChangeHandler(StepperInputChanged)
    #stepper.setOnPositionChangeHandler(StepperPositionChanged)
    #stepper.setOnVelocityChangeHandler(StepperVelocityChanged)

def setup_limit(stepper,index,acceleration,current,velocity):
    stepper.setAcceleration(index,acceleration)
    stepper.setCurrentLimit(index,current)
    stepper.setVelocityLimit(index,velocity)

def step2angel(step,motor_index):
    if motor_index == 1:
        fraction = float(step)/STEPPER_77_CIRCLE
        degree = 360*fraction
        return degree
    else:
        fraction = float(step)/STEPPER_75_CIRCLE
        degree = 360*fraction
        return degree

def angel2step(angel,motor_index):
        if motor_index == 1:
            step = int(angel/STEPPER_77_STEP*16)
            return step
        else:
            step = int(angel/STEPPER_75_STEP*16)
            return step

def stepper_init():
    #Create the two steppers
    try:
        S1 = Stepper()
        S2 = Stepper()
    except RuntimeError as e:
        print("Runtime Exception:%s"% e.details)
        print("Exiting....")
        exit(1)

    #Setup the Handler
    try:
        setup_handeller(S1)
        setup_handeller(S2)
    except RuntimeError as e:
        print("Runtime Exception:%s"% e.details)
        print("Exiting....")
        exit(1)

    print("Opening Phidget Object")

    #Open the phidget device
    try:
        S1.openPhidget(serial = Phidget_77)
        S2.openPhidget(serial = Phidget_75)
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Exiting....")
        exit(1)

    print("Waiting for attach....")

    try:
        S1.waitForAttach(10000)
        S2.waitForAttach(10000)
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        try:
            S1.closePhidget()
            S2.closePhidget()
        except PhidgetException as e:
            print("Phidget Exception %i: %s" % (e.code, e.details))
            print("Exiting....")
            exit(1)
        print("Exiting....")
        exit(1)
    print("initialization done")
    return S1,S2