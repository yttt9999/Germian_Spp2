__author__ = 'germain'

from stepper_motor_setup import *
from time import sleep
import math
import numpy as np

STEPPER_77_CIRCLE = 16457
STEPPER_75_CIRCLE = 85970
STEPPER_77_STEP = 0.35
STEPPER_75_STEP = 0.067
Phidget_77 = 398177
Phidget_75 = 398175

def drange(start,stop,step):
    r = start
    if step < 0:
        while r > stop:
            yield r
            r += step
    else:
        while r < stop:
            yield r
            r += step

def rad2degree(rad):
    degree = float (rad/(2.0*math.pi))*360
    return degree

if __name__ == "__main__":
    ################Initialize Stepper Motors#####################
    [Stepper_1,Stepper_2] = stepper_init()
    #Setup the current position as zero position
    Stepper_1.setCurrentPosition(0,0)
    Stepper_2.setCurrentPosition(0,0)
    #Engage the Stepper Motors
    Stepper_1.setEngaged(0,True)
    Stepper_2.setEngaged(0,True)
    #Setup speed, acceleration, and current
    setup_limit(Stepper_1,0,1645*10,1.5,1645*10)#speed normally 4000
    setup_limit(Stepper_2,0,8597*10,1.0,8597*10)

    Draw_circle = 1
    Draw_line_1 =2
    Draw_line_2 =3
    Draw_mode = 2

    line_dis_ratio=0.5
    initial_theta=math.pi/6  #rad
    initial_hight=1550 #mm
    real_world_r=initial_hight*math.tan(initial_theta)
    print real_world_r
    counter = 0
    sleep(2)

    if Draw_mode == Draw_circle:
    #############Main Make it move in circle line##################
        try:
            for i in drange(0,2*math.pi,math.pi/1000):
                signal_1 = 500*math.sin(i)
                signal_2 = 2000*math.cos(i)
                #signal_1 = 500*math.sin(i)
                #signal_2 = 2000*math.cos(i)

                Stepper_1.setTargetPosition(0,int(signal_1))
                Stepper_2.setTargetPosition(0,int(signal_2))
                sleep(0.001)

        except KeyboardInterrupt:
            pass
    else:
        try:
            while True:
                signal_counter = 0
                signal_1 = np.array([0])
                signal_2 = np.array([0])
                for i in drange(0,0.25*math.pi,math.pi/250):
                    signal_1 = np.append(signal_1,int(angel2step(rad2degree(i),1)))
                    signal_2 = np.append(signal_2,int(angel2step(rad2degree(math.atan((real_world_r/math.cos(i))/initial_hight)-initial_theta),2)))
                    signal_counter = signal_counter + 1


                for i in range(1,signal_counter,1):
                    Stepper_1.setTargetPosition(0,signal_1[i])
                    Stepper_2.setTargetPosition(0,signal_2[i])
                    # print signal_1,signal_2
                    sleep(0.01)

                while (Stepper_2.getCurrentPosition(0) != Stepper_2.getTargetPosition(0)):
                    pass
                while (Stepper_1.getCurrentPosition(0) != Stepper_1.getTargetPosition(0)):
                    pass

                sleep(1)


                for i in range(1,signal_counter,1):
                    Stepper_1.setTargetPosition(0,signal_1[signal_counter-i])
                    Stepper_2.setTargetPosition(0,signal_2[signal_counter-i])
                    # print signal_1,signal_2
                    sleep(0.01)


                while (Stepper_2.getCurrentPosition(0) != Stepper_2.getTargetPosition(0)):
                    pass
                while (Stepper_1.getCurrentPosition(0) != Stepper_1.getTargetPosition(0)):
                    pass















                # signal_1 = angel2step(rad2degree(i),1)
                # # change_linefor_s2 = real_world_r/math.cos(i)-real_world_r
                # signal_2 = angel2step(rad2degree(math.atan((real_world_r/math.cos(i))/initial_hight)-initial_theta),2)
                # #print step2angel(signal_2,2)
                # Stepper_1.setTargetPosition(0,int(signal_1))
                # # while (Stepper_1.getCurrentPosition(0) != Stepper_1.getTargetPosition(0)):
                # #     if abs(Stepper_1.getCurrentPosition(0) - Stepper_1.getTargetPosition(0)) < 5:
                # #         target_pos = Stepper_1.getTargetPosition(0)
                # #         target_pos = target_pos+10
                # #         Stepper_1.setTargetPosition(0,target_pos)
                # #         counter = counter + 1
                # #     else:
                # #         pass
                # #     if counter == 3:
                # #         counter = 0
                # #         break
                # #     else:
                # #         pass
                #
                # Stepper_2.setTargetPosition(0,int(signal_2))
                # sleep(0.01)


        except KeyboardInterrupt:
            pass
    ################Terminate camera and Motors#####################
    try:
        while (Stepper_2.getCurrentPosition(0) != Stepper_2.getTargetPosition(0)):
            pass
        while (Stepper_1.getCurrentPosition(0) != Stepper_1.getTargetPosition(0)):
            pass
        #sleep(3)
        #Move to the original spot
        Stepper_1.setTargetPosition(0,0)
        Stepper_2.setTargetPosition(0,0)
        sleep(1)
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
