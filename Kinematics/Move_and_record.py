__author__ = 'germain'

import os
import flycapture2 as fc2
import numpy as np
import cv2
import pygame
from time import sleep
from numpy import linalg as LA
from pygame.locals import *
from stepper_motor_setup import *
from get_key import *
import math

from camera_setup import *

green = (0,255,0)
STEPPER_bottom_CIRCLE = 16457
STEPPER_upper_CIRCLE = 85970

class Move_and_record():
    def __init__(self):
        # pygame.init()
        [self.Stepper_bottom, self.Stepper_upper] = stepper_init()
        self.Stepper_bottom.setEngaged(0,True)
        self.Stepper_upper.setEngaged(0,True)
        self.Stepper_bottom.setCurrentPosition(0,0)
        self.Stepper_upper.setCurrentPosition(0,0)
        setup_limit(self.Stepper_bottom,0,STEPPER_bottom_CIRCLE*1,1.2,STEPPER_bottom_CIRCLE*0.1)
        setup_limit(self.Stepper_upper,0,STEPPER_upper_CIRCLE*1,0.6,STEPPER_upper_CIRCLE*0.1)
        self.MyFlycam = flycamera_init()
        self.MyFlycam.start_capture()
        self.Stepper_bottom_position = 0
        self.Stepper_upper_position = 0
        self.camera_pos = np.array([(0,0)])
        self.motor_pos = np.array([(0,0)])
        self.center_pos = np.array([(0,0)])
        self.Laser_point_Loc = np.array([])
        self.Laser_point_val = np.array([])
        self.auto_bottom = 15
        self.auto_upper = 12



    def image_preproces(self):
        init_image = undistort_image(self.MyFlycam)
        [image,H] = four_pts_transormation(init_image,reference_4_points)
        self.gray_image = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        (T,self.image_thre) = cv2.threshold(self.gray_image,200,255,cv2.THRESH_BINARY)

    def update_laser_loc(self):
        self.image_preproces()
        (self.Laser_point_val,self.Laser_point_Loc) = find_brightest_point(self.image_thre,True)

    def move_motor(self,index): #"1" for bottom, "2" for upper
        if index == 1:
            self.Stepper_bottom.setTargetPosition(0,self.Stepper_bottom_position)
            # print self.Stepper_bottom_position
        elif index == 2:
            self.Stepper_upper.setTargetPosition(0,self.Stepper_upper_position)
            # print self.Stepper_upper_position

    def keyboard_control(self):
        while True:
            key = getkey()
            if key == "d":
                self.Stepper_bottom_position = self.Stepper_bottom_position + angel2step(0.5,1)
                self.move_motor(1)
                print "Right"
            if key == "a":
                self.Stepper_bottom_position = self.Stepper_bottom_position - angel2step(0.5,1)
                self.move_motor(1)
                print "Left"
            if key == "w":
                self.Stepper_upper_position = self.Stepper_upper_position - angel2step(0.3,2)
                self.move_motor(2)
                print "Up"
            if key == "s":
                self.Stepper_upper_position = self.Stepper_upper_position + angel2step(0.3,2)
                self.move_motor(2)
                print "Down"
            if key == "z":
                self.save_pos()
                print "save one point"
            if key == "p":
                self.auto_cali()
                # np.save(os.path.join('data', 'camera_pos'),self.camera_pos)
                # np.save(os.path.join('data', 'motor_pos'),self.motor_pos)
                np.save(os.path.join('data', 'camera_pos'),self.camera_pos)
                np.save(os.path.join('data', 'motor_pos'),self.motor_pos)
                image = self.pos_drawing()
                # self.center_calcu()
                # self.center_calcu()
            if key == "m":
                np.save(os.path.join('data', 'camera_pos'),self.camera_pos)
                np.save(os.path.join('data', 'motor_pos'),self.motor_pos)
                np.save(os.path.join('data', 'center_pos'),self.center_pos)
                print self.camera_pos
                print self.motor_pos
            if key == "l":
                self.camera_pos = np.load(os.path.join('data', 'camera_pos'))
                self.motor_pos = np.load(os.path.join('data', 'motor_pos'))
                self.center_pos = np.load(os.path.join('data', 'center_pos'))
                image = self.pos_drawing()
            if key == "i":
                self.line_drawing(300,300,400,400)
            if key == "q":
                break




    def save_pos(self):
        motor_pos = np.array([(step2angel(self.Stepper_bottom_position,1),step2angel(self.Stepper_upper_position,2))])
        self.motor_pos = np.append(self.motor_pos, motor_pos,axis=0)
        self.update_laser_loc()
        self.camera_pos = np.append(self.camera_pos, self.Laser_point_Loc,axis=0)

    def auto_cali(self):
        self.Stepper_bottom.setCurrentPosition(0,0)
        self.Stepper_upper.setCurrentPosition(0,0)
        self.Stepper_bottom_position = 0
        self.Stepper_upper_position = 0
        for j in range(self.auto_upper):
            for i in range(self.auto_bottom):
                 self.Stepper_bottom_position = self.Stepper_bottom_position + angel2step(3,1)
                 self.Stepper_bottom.setTargetPosition(0,self.Stepper_bottom_position)
                 while (self.Stepper_bottom.getCurrentPosition(0) != self.Stepper_bottom.getTargetPosition(0)):
                    pass
                 sleep(0.2)
                 self.save_pos()
            self.Stepper_bottom_position = 0
            self.Stepper_upper_position = self.Stepper_upper_position + angel2step(2,2)
            self.Stepper_upper.setTargetPosition(0,self.Stepper_upper_position)
            while (self.Stepper_upper.getCurrentPosition(0) != self.Stepper_upper.getTargetPosition(0)):
                    pass

        self.Stepper_upper.setTargetPosition(0,0)
        self.Stepper_bottom.setTargetPosition(0,0)
        self.motor_pos = np.delete(self.motor_pos,0,0)
        self.camera_pos = np.delete(self.camera_pos,0,0)
        # print self.camera_pos
        # print self.motor_pos

    def cross_calcu(self,point0,point1,point2,point3):  #should be numpy array each contain 2 data(x,y)
        k1 = float((point0[1]-point1[1])/(point0[0]-point1[0]))
        k2 = float((point2[1]-point3[1])/(point2[0]-point3[0]))
        cross_x = float((k1*point0[0]-k2*point2[0]+point2[1]-point0[1])/(k1-k2))
        cross_y = point0[1]+(cross_x-point0[0])*k1
        cross_point = np.array([(cross_x,cross_y)])

        return cross_point

    def center_calcu(self):
        motor_pos = self.motor_pos
        camera_pos =self.camera_pos
        for j in range(self.auto_upper-1):
            for i in range(self.auto_bottom-1):
                self.center_pos = np.append(self.center_pos,self.cross_calcu(camera_pos[i+self.auto_bottom*j],camera_pos[self.auto_bottom*(1+j)+i],camera_pos[self.auto_bottom*j+1+i],camera_pos[self.auto_bottom*(1+j)+i+1]),axis=0)
        self.center_pos = np.delete(self.center_pos,0,0)
        print self.center_pos


    def line_drawing(self,startpoint_x,startpoint_y,endpoint_x,endpoint_y):
        line_interp_x = np.linspace(startpoint_x,endpoint_x,50)
        line_interp_y = np.linspace(startpoint_y,endpoint_y,50)
        motor_move = np.array([(0,0)])
        for i in range(len(line_interp_x)):
            motor_move = np.append(motor_move,self.find_corres_theta(np.array([(line_interp_x[i],line_interp_y[i])])),axis=0)
        print motor_move


    def pos_drawing(self):
        init_image = undistort_image(self.MyFlycam)
        [image,H] = four_pts_transormation(init_image,reference_4_points)
        for i in range(len(self.camera_pos)):
            cv2.circle(image,(self.camera_pos[i][0],self.camera_pos[i][1]),2,green)
        cv2.imshow("circle",image)
        while True:
            if cv2.waitKey(1) & 0xFF == ord("q"):
                cv2.imwrite(os.path.join('data','autocali.jpg'),image)
                cv2.destroyAllWindows()
                break
        return image

    def find_nearsest_point(self,point):
        camera_pos =self.camera_pos
        point_distance = np.array([])
        for j in range(self.auto_upper-1):
            for i in range(self.auto_bottom-1):
                point_distance = np.append(point_distance,LA.norm(point - camera_pos[i+self.auto_bottom*j]) + \
                                           LA.norm(point - camera_pos[self.auto_bottom*(1+j)+i]) + \
                                           LA.norm(point - camera_pos[self.auto_bottom*j+1+i]) + \
                                           LA.norm(point - camera_pos[self.auto_bottom*(1+j)+i+1]))
        # for i in range(len(self.camera_pos)):
        #     point_distance = np.append(point_distance,LA.norm(point-self.camera_pos[i]))
        return np.argmin(point_distance)

    def find_corres_theta(self,point):
        camera_pos = self.camera_pos
        motor_pos = self.motor_pos
        nearest_index = self.find_nearsest_point(point)
        circle_center = self.center_pos[nearest_index]
        reference_point_index = (self.auto_bottom * nearest_index / (self.auto_bottom-1)) + nearest_index % (self.auto_bottom-1)
        reference_point_index_1 = reference_point_index + self.auto_bottom  #for calculating upper motor movement
        camera_ref_pos = camera_pos[reference_point_index]
        camera_ref_pos_1 = camera_pos[reference_point_index_1]   #for calculating upper motor movement
        motor_ref_pos = motor_pos[reference_point_index]
        motor_ref_pos_1 = motor_pos[reference_point_index_1]    #for calculating upper motor movement
        refer_angle = float(math.atan((camera_ref_pos[0] - circle_center[0]) / (camera_ref_pos[1] - circle_center[1])))
        real_angle = float(math.atan((point[0] - circle_center[0]) / (point[1] - circle_center[1])))
        bottom_step = angel2step(math.degrees(refer_angle - real_angle) + motor_ref_pos[0],1)
        refer_dis = float(math.sqrt((camera_ref_pos[0] - circle_center[0])**2+(camera_ref_pos[1] - circle_center[1])**2))
        refer_dis_1 = float(math.sqrt((camera_ref_pos_1[0] - circle_center[0])**2+(camera_ref_pos_1[1] - circle_center[1])**2))
        theta2 = motor_ref_pos_1[1] - motor_ref_pos[1]
        a = ((refer_dis - refer_dis_1)**2)/(2.0*math.tan(theta2)) - math.sqrt(refer_dis_1*refer_dis)
        theta1 = math.degrees(math.atan(refer_dis_1/a))
        real_dis = float(math.sqrt((point[0] - circle_center[0])**2+(point[1] - circle_center[1])**2))
        move_real_theta = math.degrees(math.atan(real_dis/a)) - theta1
        upper_step = angel2step(motor_ref_pos_1[1]-move_real_theta,2)
        return np.array([(bottom_step,upper_step)])






def main():
    MR = Move_and_record()
    # pygame.key.set_repeat(1, 20)
    MR.keyboard_control()


if __name__ == "__main__":
    main()