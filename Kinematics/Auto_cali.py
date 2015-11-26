__author__ = 'germain'

import flycapture2 as fc2
import numpy as np
import cv2
import pygame
from time import sleep
from stepper_motor_setup import *
from camera_setup import *

green = (0,255,0)
STEPPER_bottom_CIRCLE = 16457
STEPPER_upper_CIRCLE = 85970


class Auto_cali():
    def __init__(self):
        self.Laser_point_Loc = np.array([(0,0)])
        self.Laser_point_val = np.array([])
        self.camera_cd = np.array([])
        self.real_world_cd = np.array([])
        self.gray_image = np.array([])
        self.image_thre = np.array([])
        self.upper_pos_change = np.array([(0,0)])
        self.bottom_pos_change = np.array([(0,0)])
        self.destination = np.array([])
        [self.Stepper_bottom, self.Stepper_upper] = stepper_init()
        self.MyFlycam = flycamera_init()
        self.MyFlycam.start_capture()

    def cross_calcu(self,point0,point1,point2,point3):  #should be numpy array each contain 2 data(x,y)
        k1 = float((point0[1]-point1[1])/(point0[0]-point1[0]))
        k2 = float((point2[1]-point3[1])/(point2[0]-point3[0]))
        cross_x = float((k1*point0[0]-k2*point2[0]+point2[1]-point0[1])/(k1-k2))
        cross_y = point0[1]+(cross_x-point0[0])*k1
        cross_point = np.array([(cross_x,cross_y)])
        print cross_point
        np.save('cross_point',cross_point)
        return cross_point

    def moving_test(self):
        self.Stepper_bottom.setCurrentPosition(0,0)
        self.Stepper_upper.setCurrentPosition(0,0)
        self.update_laser_loc()
        Test_point = self.Laser_point_Loc


        self.Stepper_upper.setTargetPosition(0,angel2step(5,2))
        # cv2.imshow("gray&thres picture of the original", self.image_thre)
        while (self.Stepper_upper.getCurrentPosition(0) != self.Stepper_upper.getTargetPosition(0)):
            pass
        sleep(1)
        self.update_laser_loc()
        Test_point = np.append(Test_point, self.Laser_point_Loc,axis=0)
        # self.upper_pos_change = upper_Laser_pos - ini_Laser_pos


        self.Stepper_bottom.setTargetPosition(0,angel2step(5,1))
        while (self.Stepper_bottom.getCurrentPosition(0) != self.Stepper_bottom.getTargetPosition(0)):
            pass
        sleep(1)
        self.update_laser_loc()
        Test_point = np.append(Test_point, self.Laser_point_Loc,axis=0)


        self.Stepper_upper.setTargetPosition(0,angel2step(0,2))
        # cv2.imshow("gray&thres picture of the original", self.image_thre)
        while (self.Stepper_upper.getCurrentPosition(0) != self.Stepper_upper.getTargetPosition(0)):
            pass
        sleep(1)
        self.update_laser_loc()
        Test_point = np.append(Test_point, self.Laser_point_Loc,axis=0)

        self.cross_calcu(Test_point[0],Test_point[1],Test_point[2],Test_point[3])
        # self.bottom_pos_change = bottom_Laser_pos - upper_Laser_pos
        # np.save('upper_pos_change',self.upper_pos_change)
        # np.save('bottom_pos_change',self.bottom_pos_change)
        self.Stepper_upper.setTargetPosition(0,0)
        self.Stepper_bottom.setTargetPosition(0,0)
        # print self.upper_pos_change, self.bottom_pos_change
        print Test_point

    def straight_line_test(self):
        self.Stepper_bottom.setCurrentPosition(0,0)
        self.Stepper_upper.setCurrentPosition(0,0)
        self.update_laser_loc()
        Test_point = self.Laser_point_Loc

        for i in range 10
            self.Stepper_upper.setTargetPosition(0,angel2step(i*4,2))
            # cv2.imshow("gray&thres picture of the original", self.image_thre)
            while (self.Stepper_upper.getCurrentPosition(0) != self.Stepper_upper.getTargetPosition(0)):
                pass
            sleep(1)
            self.update_laser_loc()
            Test_point = np.append(Test_point, self.Laser_point_Loc,axis=0)
        print Test_point

    def image_preproces(self):
        image = undistort_image(self.MyFlycam)
        # [image,H] = four_pts_transormation(init_image,reference_4_points)
        self.gray_image = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        (T,self.image_thre) = cv2.threshold(self.gray_image,200,255,cv2.THRESH_BINARY)

    def update_laser_loc(self):
        self.image_preproces()
        (self.Laser_point_val,self.Laser_point_Loc) = find_brightest_point(self.image_thre,True)


    def move_to_des(self,des):
        self.update_laser_loc()
        diff = self.Laser_point_Loc - des
        setup_limit(self.Stepper_bottom,0,STEPPER_bottom_CIRCLE*1,1.2,STEPPER_bottom_CIRCLE*0.1)
        setup_limit(self.Stepper_upper,0,STEPPER_upper_CIRCLE*1,0.6,STEPPER_upper_CIRCLE*0.1)
        ini_step_bottom = 0
        ini_step_upper = 0
        while abs(diff[0][0]) > 5 or abs(diff[0][1]) >5:
            self.update_laser_loc()
            diff = self.Laser_point_Loc - des
            self.Stepper_bottom.setTargetPosition(0,diff[0][0]*10+ini_step_bottom)




    





################capture frame, do distort, extract black point###########################################



def main():

###############Initialize the Camera#########################
        # MyFlycam = flycamera_init()
        # MyFlycam.start_capture()
    ################Initialize Stepper Motors#####################
        Ac = Auto_cali()
        #Setup the current position as zero position
        Ac.Stepper_bottom.setCurrentPosition(0,0)
        Ac.Stepper_upper.setCurrentPosition(0,0)
        #Engage the Stepper Motors
        Ac.Stepper_bottom.setEngaged(0,True)
        Ac.Stepper_upper.setEngaged(0,True)
        #Setup speed, acceleration, and current
        setup_limit(Ac.Stepper_bottom,0,STEPPER_bottom_CIRCLE*10,1.2,STEPPER_bottom_CIRCLE)#acceleration,current & velocity
        setup_limit(Ac.Stepper_upper,0,STEPPER_upper_CIRCLE*10,0.6,STEPPER_upper_CIRCLE)

        sleep(2)
        whether_test = raw_input("1 for Processing moving test or 2 for load from existing file:")
        if whether_test == "1":
            # Ac.moving_test()
            Ac.straight_line_test()
        else :
            Ac.upper_pos_change = np.load('upper_pos_change')
            Ac.bottom_pos_change = np.load('bottom_pos_change')
    ##############################Main Program is designed as following#########################
        # while True:
        # #     self.image_preproces()
        # #
        # #
        # #
        # #     cv2.imshow("gray&thres picture of the original", image_thre)
        #     if cv2.waitKey(1) & 0xFF == ord("q"):
        #         break
        cv2.destroyAllWindows()
        Ac.MyFlycam.stop_capture()
        Ac.MyFlycam.disconnect()







################move the motor to the every point and record the theta of bottom motor and upper motor#####













################generate the table of connection of theta and the real world################################



if __name__ == "__main__":
    main()