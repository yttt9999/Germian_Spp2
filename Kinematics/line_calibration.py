import stepper_motor_setup import *
import math
import cv2
import numpy as np
import argparse
import time
from camera_setup import *
'''
green = (0, 255, 0)
ap = argparse.ArgumentParser()
ap.add_argument("-v","--video",help = "path ot the video")
args = vars(ap.parse_args())

camera = cv2.VideoCapture(args["video"])

while True:
	(grabbed, frame) = camera.read()
	if not grabbed:
	    break

	frameClone = frame.copy()
	frame_final_show = frame.copy()
	frameClone = frameClone [50:frameClone.shape[0]-70, 120:frameClone.shape[1]-50]
	[light_intencity,light_position] = find_brightest_point(frameClone)
	cv2.circle(frame_final_show,(light_position[0]+120, light_position[1]+50),5,green)
	#cv2.circle(frameClone,light_position,5,green)
	cv2.imshow("light_catch",frame_final_show)
	if cv2.waitKey(1) & 0xFF == ord("q"):
	    print frame.shape[0], frame.shape[1]
	    break
	time.sleep(0.01)
	#[dark_intencity,dark_position] = find_darkest_point(frame)
	#print light_position
	#cv2.rectangle(frameClone, light_position, )
camera.release()
cv2.destroyAllWindows()
'''
if __name__ == "__main__":
    Myflycam = flycamera_init()
    Myflycam.start_capture()

    [Stepper_1] = stepper_init()
	[Stepper_2] = stepper_init()
    #Setup the current position as zero position
    Stepper_1.setCurrentPosition(0,0)
    Stepper_2.setCurrentPosition(0,0)
    #Engage the Stepper Motors
    Stepper_1.setEngaged(0,True)
    Stepper_2.setEngaged(0,True)
    #Setup speed, acceleration, and current
    setup_limit(Stepper_1,0,100000,1.2,16457)#speed normally 4000
    setup_limit(Stepper_2,0,2000000,0.6,180000)

    sleep(2)

    try:
        #Move to the original spot
        Stepper_1.setTargetPosition(0,0)
        Stepper_2.setTargetPosition(0,0)
        sleep(3)
        #Turn off the engage of the motors
        Stepper_1.setEngaged(0,False)
        Stepper_2.setEngaged(0,False)
        sleep(1)
        Stepper_1.closePhidget()
        Stepper_2.closePhidget()
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Exiting....")
        exit(1)
    exit(0)

       
