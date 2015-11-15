__author__ = 'kevin'
import numpy as np
from math import *
#Define couple of caractors
XP1 = 16.51
YP1 = 3
ZP1 = 137.16
Y_P1_P2 = 3.5
Z_P1_P2 = 2
X_P2_P3 = 4.5
Y_P2_P3 = -0.5
Z_P2_P3 = 0
MIN_1 = 0
MAX_1 = 181
MIN_2 = 0
MAX_2 = 91
HEAD_POINT_X = 21
HEAD_POINT_Y = 5.5
HEAD_POINT_Z = 141
COMP_ANGLE = 0

def kinematics_frame_transformation(initial_point,*RS):
    point = initial_point
    for R in RS:
        point = R.dot(point)
    return point

def rad_to_deg(rad):
    deg = rad/pi*180
    return deg

def deg_to_rad(deg):
    rad = deg*pi/180
    return rad
def angle_limit(theta,min,max):
    if theta < min or theta > max:
        return False
    else:
        return True

def forward_kinematics(theta_1,theta_2):
    theta_1 = -deg_to_rad(theta_1)
    theta_2 = -deg_to_rad(theta_2)
    T_1_0 = np.matrix([[cos(theta_1),-sin(theta_1),0,XP1],
                           [sin(theta_1),cos(theta_1),0,YP1],
                           [0,0,1,ZP1],
                           [0,0,0,1]])
        #print T_1_0
    T_2_1 = np.matrix([[cos(theta_2),0,sin(theta_2),0],
                           [0,1,0,Y_P1_P2],
                           [-sin(theta_2),0,cos(theta_2),Z_P1_P2],
                           [0,0,0,1]])
        #print T_2_1
    T_3_2 = np.matrix([[cos(COMP_ANGLE),0,sin(COMP_ANGLE),X_P2_P3],
                           [0,1,0,Y_P2_P3],
                           [-sin(COMP_ANGLE),0,cos(COMP_ANGLE),Z_P2_P3],
                           [0,0,0,1]])
        #print T_3_2
    p3 = np.array([[0],
                       [0],
                       [0],
                       [1]])
    p3_orientation = np.array([[1],
                                   [0],
                                   [0],
                                   [1]])
    p3_0 = kinematics_frame_transformation(p3,T_3_2,T_2_1,T_1_0)
    p3_orientation_0 = kinematics_frame_transformation(p3_orientation,T_3_2,T_2_1,T_1_0)-p3_0

    gain = p3_0[2,0]/p3_orientation_0[2,0]
    ground_coord = p3_0-gain*p3_orientation_0
    return ground_coord


def laser_point (theta_1,theta_2):
    theta_1 = -deg_to_rad(theta_1)
    theta_2 = -deg_to_rad(theta_2)
    T_1_0 = np.matrix([[cos(theta_1),-sin(theta_1),0,XP1],
                           [sin(theta_1),cos(theta_1),0,YP1],
                           [0,0,1,ZP1],
                           [0,0,0,1]])
        #print T_1_0
    T_2_1 = np.matrix([[cos(theta_2),0,sin(theta_2),0],
                           [0,1,0,Y_P1_P2],
                           [-sin(theta_2),0,cos(theta_2),Z_P1_P2],
                           [0,0,0,1]])
        #print T_2_1
    T_3_2 = np.matrix([[cos(COMP_ANGLE),0,sin(COMP_ANGLE),X_P2_P3],
                           [0,1,0,Y_P2_P3],
                           [-sin(COMP_ANGLE),0,cos(COMP_ANGLE),Z_P2_P3],
                           [0,0,0,1]])
        #print T_3_2
    p3 = np.array([[0],
                       [0],
                       [0],
                       [1]])
    p3_orientation = np.array([[1],
                                   [0],
                                   [0],
                                   [1]])
    p3_0 = kinematics_frame_transformation(p3,T_3_2,T_2_1,T_1_0)
    return p3_0

def inverse_kinematics(future_point,laser_point):
    f_x = future_point[0,0]
    f_y = future_point[1,0]

    l_x = laser_point[0,0]
    l_y = laser_point[1,0]
    l_z = laser_point[2,0]
    future_angle_1 = -rad_to_deg(atan2(f_y-l_y,f_x-l_x))
    future_angle_2 = -rad_to_deg(atan2(l_z,sqrt((f_x-l_x)*(f_x-l_x)+(f_y-l_y)*(f_y-l_y))))
    return future_angle_1,future_angle_2


