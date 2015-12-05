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
from scipy import stats
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
        setup_limit(self.Stepper_bottom,0,STEPPER_bottom_CIRCLE*0.3,1.2,STEPPER_bottom_CIRCLE*0.1)
        setup_limit(self.Stepper_upper,0,STEPPER_upper_CIRCLE*0.3,0.6,STEPPER_upper_CIRCLE*0.1)
        self.MyFlycam = flycamera_init()
        self.MyFlycam.start_capture()
        self.Stepper_bottom_position = 0
        self.Stepper_upper_position = 0
        self.camera_pos = np.array([(0,0)])
        self.motor_pos = np.array([(0,0)])
        self.center_pos = np.array([(0,0)])
        self.temp_slope_intercept =  np.array([(0,0)])
        self.Laser_point_Loc = np.array([])
        self.Laser_point_val = np.array([])
        self.delete_list = []
        self.auto_bottom = 30
        self.auto_upper = 24
        self.direction_flag = 0
        self.area_limi_x_min = 40
        self.area_limi_x_max = 230
        self.area_limi_y_min = 20
        self.area_limi_y_max = 160




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
                self.Stepper_bottom_position = self.Stepper_bottom_position + angel2step(0.1,1)
                self.move_motor(1)
                print "Right"
            if key == "a":
                self.Stepper_bottom_position = self.Stepper_bottom_position - angel2step(0.1,1)
                self.move_motor(1)
                print "Left"
            if key == "w":
                self.Stepper_upper_position = self.Stepper_upper_position - angel2step(0.1,2)
                self.move_motor(2)
                print "Up"
            if key == "s":
                self.Stepper_upper_position = self.Stepper_upper_position + angel2step(0.1,2)
                self.move_motor(2)
                print "Down"
            if key == "z":
                # self.save_pos()
                self.center_calcu()
                print "center_calcu_done"
                print self.center_pos
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
                print "Save Done"
                # print self.camera_pos
                # print self.motor_pos
            if key == "l":
                self.camera_pos = np.load(os.path.join('data', 'camera_pos.npy'))
                self.motor_pos = np.load(os.path.join('data', 'motor_pos.npy'))
                # self.center_pos = np.load(os.path.join('data', 'center_pos'))
                # image = self.pos_drawing()
                print "Load Down"
            if key == "i":
                self.line_regress()
                intersect_point = self.intersect_finding(40.0,60.0,150.0,170.0)
                move_point = self.find_nearest_2_point(intersect_point)
                self.point_move(move_point)
                sleep(2)
                setup_limit(self.Stepper_bottom,0,STEPPER_bottom_CIRCLE*0.3,1.2,STEPPER_bottom_CIRCLE*0.1)
                setup_limit(self.Stepper_upper,0,STEPPER_upper_CIRCLE*0.3,0.6,STEPPER_upper_CIRCLE*0.1)
                self.Stepper_bottom.setTargetPosition(0,0)
                self.Stepper_upper.setTargetPosition(0,0)
                sleep(2)
                intersect_point = self.intersect_finding(40.0,60.0,180.0,60.0)
                move_point = self.find_nearest_2_point(intersect_point)
                self.point_move(move_point)
                sleep(2)
                setup_limit(self.Stepper_bottom,0,STEPPER_bottom_CIRCLE*0.3,1.2,STEPPER_bottom_CIRCLE*0.1)
                setup_limit(self.Stepper_upper,0,STEPPER_upper_CIRCLE*0.3,0.6,STEPPER_upper_CIRCLE*0.1)
                self.Stepper_bottom.setTargetPosition(0,0)
                self.Stepper_upper.setTargetPosition(0,0)
                sleep(2)
                intersect_point = self.intersect_finding(40.0,60.0,150.0,120.0)
                move_point = self.find_nearest_2_point(intersect_point)
                self.point_move(move_point)
                # intersect_point = self.intersect_finding(100.0,150.0,140.0,100.0)
                # move_point = self.find_nesrst_2_point(intersect_point)
                # self.point_move(move_point)
                # intersect_point = self.intersect_finding(140.0,100.0,60.0,100.0)
                # move_point = self.find_nesrst_2_point(intersect_point)
                # self.point_move(move_point)
            if key == "v":
                slope = 1.038
                self.line_regress()
                intersect_point = self.intersect_finding_mode2(slope,135.0,50.0)
                move_point = self.find_nearest_2_point(intersect_point)
                intersect_point = np.delete(intersect_point,self.delete_list,0)
                start_x = intersect_point[len(intersect_point)-1][0]
                start_y = intersect_point[len(intersect_point)-1][1]
                print move_point
                for i in range(5):
                    slope = -1.0 * slope
                    intersect_point = self.intersect_finding_mode2(slope,start_x,start_y)
                    move_point = self.find_nearest_2_point(intersect_point)
                    self.point_move(move_point)
                    intersect_point = np.delete(intersect_point,self.delete_list,0)
                    if abs(start_x - intersect_point[len(intersect_point)-1][0]) < 0.1:
                        start_x = intersect_point[0][0]
                        start_y = intersect_point[0][1]
                    else:
                        start_x = intersect_point[len(intersect_point)-1][0]
                        start_y = intersect_point[len(intersect_point)-1][1]
                print move_point

            if key == "o":
                self.Stepper_bottom.setCurrentPosition(0,0)
                self.Stepper_upper.setCurrentPosition(0,0)
                self.Stepper_bottom_position = 0
                self.Stepper_upper_position = 0
                print "Set origin down"
            if key == "b":
                setup_limit(self.Stepper_bottom,0,STEPPER_bottom_CIRCLE*0.3,1.2,STEPPER_bottom_CIRCLE*0.1)
                setup_limit(self.Stepper_upper,0,STEPPER_upper_CIRCLE*0.3,0.6,STEPPER_upper_CIRCLE*0.1)
                self.Stepper_bottom.setTargetPosition(0,0)
                self.Stepper_upper.setTargetPosition(0,0)
                self.Stepper_bottom_position = 0
                self.Stepper_upper_position = 0
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
                 self.Stepper_bottom_position = self.Stepper_bottom_position + angel2step(1.15,1)
                 self.Stepper_bottom.setTargetPosition(0,self.Stepper_bottom_position)
                 while (self.Stepper_bottom.getCurrentPosition(0) != self.Stepper_bottom.getTargetPosition(0)):
                    pass
                 sleep(0.3)
                 self.save_pos()
            self.Stepper_bottom_position = 0
            self.Stepper_upper_position = self.Stepper_upper_position + angel2step(1,2)
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
        x1=point0[0]
        y1=point0[1]
        x2=point1[0]
        y2=point1[1]
        x3=point2[0]
        y3=point2[1]
        x4=point3[0]
        y4=point3[1]

        k1 = float((point0[1]-point1[1])/(point0[0]-point1[0]))
        k2 = float((point2[1]-point3[1])/(point2[0]-point3[0]))
        if k1 == k2:
            cross_x = -1
            cross_y = -1
        else:
            # cross_x = float((k1*point0[0]-k2*point2[0]+point2[1]-point0[1])/(k1-k2))
            # cross_y = point0[1]+(cross_x-point0[0])*k1
            cross_x = float(((x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4))) / float(((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4)))
            cross_y = float(((x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4))) / float(((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4)))
        if cross_x > 500 or cross_y > 500 or cross_x < 0 or cross_y < 0:
            cross_x = -1
            cross_y = -1
        else:
            pass
        cross_point = np.array([(cross_x,cross_y)])

        return cross_point

    def center_calcu(self):
        motor_pos = self.motor_pos
        camera_pos = self.camera_pos
        counter = 0
        for j in range(self.auto_upper-1):
            for i in range(self.auto_bottom-1):
                counter = counter + 1
                cross_cal_res = self.cross_calcu(camera_pos[i+self.auto_bottom*j],camera_pos[self.auto_bottom*(1+j)+i],camera_pos[self.auto_bottom*j+1+i],camera_pos[self.auto_bottom*(1+j)+i+1])
                if cross_cal_res[0][0] == -1:
                    self.center_pos = np.append(self.center_pos,np.array([(self.center_pos[counter-1][0],self.center_pos[counter-1][1])]),axis=0)
                else:
                    self.center_pos = np.append(self.center_pos,cross_cal_res,axis=0)
        self.center_pos = np.delete(self.center_pos,0,0)
        # print self.center_pos


    def line_drawing(self,startpoint_x,startpoint_y,endpoint_x,endpoint_y):
        line_interp_x = np.linspace(startpoint_x,endpoint_x,10)
        line_interp_y = np.linspace(startpoint_y,endpoint_y,10)
        motor_move = np.array([(0,0)])
        for i in range(len(line_interp_x)):
            motor_move = np.append(motor_move,self.find_corres_theta(np.array([(line_interp_x[i],line_interp_y[i])])),axis=0)
        print "motor move point",motor_move


    def pos_drawing(self):
        init_image = undistort_image(self.MyFlycam)
        [image,H] = four_pts_transormation(init_image,reference_4_points)
        for i in range(len(self.camera_pos)):
            cv2.circle(image,(self.camera_pos[i][0],self.camera_pos[i][1]),2,green)
        cv2.imshow("circle",image)
        while True:
            if cv2.waitKey(1) & 0xFF == ord("s"):
                cv2.imwrite(os.path.join('data','autocali.jpg'),image)
                break
        cv2.destroyAllWindows()
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
        real_angle = float(math.atan((point[0][0] - circle_center[0]) / (point[0][1] - circle_center[1])))
        bottom_step = angel2step(math.degrees(refer_angle - real_angle) + motor_ref_pos[0],1)
        refer_dis = float(math.sqrt((camera_ref_pos[0] - circle_center[0])**2+(camera_ref_pos[1] - circle_center[1])**2))
        refer_dis_1 = float(math.sqrt((camera_ref_pos_1[0] - circle_center[0])**2+(camera_ref_pos_1[1] - circle_center[1])**2))
        theta2 = motor_ref_pos_1[1] - motor_ref_pos[1]
        a = ((refer_dis - refer_dis_1)**2)/(2.0*math.tan(theta2)) - math.sqrt(refer_dis_1*refer_dis)
        theta1 = math.degrees(math.atan(refer_dis_1/a))
        real_dis = float(math.sqrt((point[0][0] - circle_center[0])**2+(point[0][1] - circle_center[1])**2))
        move_real_theta = math.degrees(math.atan(real_dis/a)) - theta1
        upper_step = angel2step(motor_ref_pos_1[1]-move_real_theta,2)
        return np.array([(bottom_step,upper_step)])

    def line_regress(self):
        self.temp_slope_intercept = np.array([(0,0)])
        temp_array_x = np.array([])
        temp_array_y = np.array([])
        camera_reshape = np.reshape(self.camera_pos,(self.auto_upper,self.auto_bottom*2))
        # print camera_reshape
        for j in range(self.auto_bottom):
            for i in range(self.auto_upper):
                temp_array_x = np.append(temp_array_x,camera_reshape[i][j*2])
                temp_array_y = np.append(temp_array_y,camera_reshape[i][j*2+1])
            slope, intercept, r_calue, p_value, std_err = stats.linregress(temp_array_x,temp_array_y)
            # temp_sl_in = np.polyfit(temp_array_x,temp_array_y,1)
            temp_array_x = np.array([])
            temp_array_y = np.array([])
            self.temp_slope_intercept = np.append(self.temp_slope_intercept,np.array([(slope,intercept)]),axis=0)
        self.temp_slope_intercept = np.delete(self.temp_slope_intercept,0,0)
        # print self.temp_slope_intercept

    def intersect_finding(self,startpoint_x,startpoint_y,endpoint_x,endpoint_y):
        self.delete_list = []
        self.direction_flag = 0
        slope = float((endpoint_y - startpoint_y) / (endpoint_x-startpoint_x))
        intercept = startpoint_y - slope*startpoint_x
        # print slope, intercept
        intercept_point = np.array([(0,0)])
        for i in range(len(self.temp_slope_intercept)):
            cross_x = float((intercept-self.temp_slope_intercept[i][1]) / (self.temp_slope_intercept[i][0] - slope))
            cross_y = float((self.temp_slope_intercept[i][0]*intercept - slope*self.temp_slope_intercept[i][1]) / (self.temp_slope_intercept[i][0] - slope))
            intercept_point = np.append(intercept_point, np.array([(cross_x,cross_y)]),axis=0)
        intercept_point = np.delete(intercept_point,0,0)
        for j in range(len(intercept_point)):
            if (intercept_point[j][0] < self.area_limi_x_min or intercept_point[j][0] > self.area_limi_x_max or intercept_point[j][1] < self.area_limi_y_min or intercept_point[j][0] > self.area_limi_y_max):
                self.delete_list.append(j)
        if startpoint_x < endpoint_x:
            self.direction_flag = -1
        else:
            self.direction_flag = 1

        # print intercept_point
        return intercept_point

    def intersect_finding_mode2(self,slope,startpoint_x,startpoint_y):
        self.delete_list = []
        self.direction_flag = 0
        # slope = float((endpoint_y - startpoint_y) / (endpoint_x-startpoint_x))
        intercept = startpoint_y - slope*startpoint_x
        # print slope, intercept
        intercept_point = np.array([(0,0)])
        for i in range(len(self.temp_slope_intercept)):
            cross_x = float((intercept-self.temp_slope_intercept[i][1]) / (self.temp_slope_intercept[i][0] - slope))
            cross_y = float((self.temp_slope_intercept[i][0]*intercept - slope*self.temp_slope_intercept[i][1]) / (self.temp_slope_intercept[i][0] - slope))
            intercept_point = np.append(intercept_point, np.array([(cross_x,cross_y)]),axis=0)
        intercept_point = np.delete(intercept_point,0,0)
        for j in range(len(intercept_point)):
            if (intercept_point[j][0] < self.area_limi_x_min or intercept_point[j][0] > self.area_limi_x_max or intercept_point[j][1] < self.area_limi_y_min or intercept_point[j][0] > self.area_limi_y_max):
                self.delete_list.append(j)
        intercept_copy = intercept_point
        intercept_copy = np.delete(intercept_copy,self.delete_list,0)
        if startpoint_x < intercept_copy[len(intercept_copy)-1][0]:
            self.direction_flag = -1
        else:
            self.direction_flag = 1

        # print intercept_point
        return intercept_point

    def find_nearest_2_point(self,point):
        camera_reshape = np.reshape(self.camera_pos,(self.auto_upper,self.auto_bottom*2))
        motor_reshape = np.reshape(self.motor_pos,(self.auto_upper,self.auto_bottom*2))
        point_distance = np.array([])
        point_index = np.array([])
        point_theta = np.array([(0,0)])
        for j in range(self.auto_bottom):
            for i in range(self.auto_upper-1):
                point_distance = np.append(point_distance,abs(point[j][1]-camera_reshape[i][2*j+1])+abs(point[j][1]-camera_reshape[i+1][2*j+1]))
            point_index = np.append(point_index,np.argmin(point_distance))
            point_distance = np.array([])
        for k in range(self.auto_bottom):
            weight = (float(camera_reshape[point_index[k]+1][2*k+1]) - point[k][1]) / float((camera_reshape[point_index[k]+1][2*k+1])-(camera_reshape[point_index[k]][2*k+1]))
            point_theta_2 = float(motor_reshape[point_index[k]+1][2*k+1]-((motor_reshape[point_index[k]+1][2*k+1]-(motor_reshape[point_index[k]][2*k+1])) * weight))
            point_theta_1 = motor_reshape[1][2*k]
            point_theta = np.append(point_theta, np.array([(point_theta_1,point_theta_2)]),axis=0)
        point_theta = np.delete(point_theta,0,0)
        point_theta = np.delete(point_theta,self.delete_list,0)
        if self.direction_flag == -1:
            pass
        elif self.direction_flag == 1:
            point_theta = np.flipud(point_theta)
        print point_theta
        return point_theta


    def point_move(self,move_point):
        for i in range(len(move_point)):
            if i == 0:
                self.Stepper_bottom_position = angel2step(move_point[i][0],1)
                self.Stepper_upper_position = angel2step(move_point[i][1],2)
                self.move_motor(1)
                self.move_motor(2)
                while (self.Stepper_upper.getCurrentPosition(0) != self.Stepper_upper.getTargetPosition(0)):
                    pass
                while (self.Stepper_bottom.getCurrentPosition(0) != self.Stepper_bottom.getTargetPosition(0)):
                    pass
                sleep(1)
            else:
                bottom_move = angel2step(abs(move_point[i][0]-move_point[i-1][0]),1)
                upper_move = angel2step(abs(move_point[i][1]-move_point[i-1][1]),2)
                if bottom_move == 0:
                    bottom_move = 100
                elif upper_move == 0:
                    upper_move = 100
                setup_limit(self.Stepper_bottom,0,bottom_move*1000,1.2,bottom_move*10)
                setup_limit(self.Stepper_upper,0,upper_move*1000,0.6,upper_move*10)
                self.Stepper_bottom_position = angel2step(move_point[i][0],1)
                self.Stepper_upper_position = angel2step(move_point[i][1],2)
                self.move_motor(1)
                self.move_motor(2)

                while (abs(self.Stepper_upper.getCurrentPosition(0) - self.Stepper_upper.getTargetPosition(0)) > 50):
                    pass
                while (abs(self.Stepper_bottom.getCurrentPosition(0) - self.Stepper_bottom.getTargetPosition(0)) > 50):
                    pass
                # print self.Stepper_bottom.getCurrentPosition(0) - self.Stepper_bottom.getTargetPosition(0)




def main():
    MR = Move_and_record()
    # pygame.key.set_repeat(1, 20)
    MR.keyboard_control()


if __name__ == "__main__":
    main()