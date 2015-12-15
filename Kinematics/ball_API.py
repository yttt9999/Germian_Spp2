import numpy as np
import cv2
from math import *
from camera_setup import find_brightest_point

PI = 3.1415926
X_U = 234
X_L = 126
Y_U = 161
Y_L = 89


class Ball(object):
    def __init__(self, init_x, init_y, theta, speed):
        # initialize a point
        self.center_x = init_x
        self.center_y = init_y
        self.theta = float(theta)/180*PI
        self.speed = speed # horizon

    def updateBall(self, img, brighest):

        height, width = img.shape
        #img = cv2.medianBlur(img,5)
        hit_x = 0
        hit_y = 0

        # flag = 1
        # while flag:
        #     flag = 0
            # print theta
        last_x = self.center_x
        last_y = self.center_y

        # brighest = find_brightest_point(gray, 1)
        self.center_x = brighest[0][0]
        self.center_y = brighest[0][1]
        
        # update theta
        self.theta = atan2(self.center_y - last_y, self.center_x - last_x)

        next_x = int(round(self.center_x + self.speed*cos(self.theta)))
        next_y = int(round(self.center_y + self.speed*sin(self.theta)))
        print (next_x, next_y)
            # if next_x < 0:
            #     self.theta = PI-self.theta
            #     flag = 1
            # elif next_x >= width:
            #     self.theta = PI-self.theta
            #     flag = 1
            # elif next_y < 0:
            #     self.theta = -self.theta
            #     flag = 1
            # elif next_y >= height:
            #     self.theta = -self.theta
            #     flag = 1
        # print (next_y, next_x)
        # print theta/PI*180
        # check if next position hits foreground
        # if img[next_y, next_x] > 0:
        hit_flag = 0
        dx = -cos(self.theta)
        dy = -sin(self.theta)
        for i in xrange(int(self.speed/5)):
            dx_i = int(round(i*dx))
            dy_i = int(round(i*dy))
            if img[next_y + dy_i, next_x + dx_i] > 0:
                # print i
                hit_flag = 1
            if hit_flag and img[next_y + dy_i, next_x + dx_i] == 0:
                break
        if hit_flag:
            hit_x = next_x + dx_i
            hit_y = next_y + dy_i

            RANGE = 10
            window_hit = img[max(hit_y-RANGE,0):min(hit_y+RANGE,height), max(hit_x-RANGE,0):min(hit_x+RANGE,width)]
            # hi_img = np.zeros((2*RANGE+1,2*RANGE+1),np.uint8)
            edges = cv2.Canny(window_hit,50,150,apertureSize = 3)
            # cv2.imshow('new', edges)
            # cv2.waitKey(0)

            lines = cv2.HoughLinesP(edges,1,np.pi/180,10)
            number  = 0
            alpha = 0
            try:
                for x1,y1,x2,y2 in lines[0]:
                    # cv2.line(hi_img,(x1,y1),(x2,y2),255,2)
                    # print (x1,y1,x2,y2)
                    alpha = (float(alpha*number) + float(atan2(y2 - y1, x2 - x1)))/float((number+1))
                    number = number + 1
                    #print number
            except:
                pass

            if number>0:
                # print alpha/PI*180
                self.theta = 2*alpha - self.theta

                # cv2.imshow('hi_img', hi_img)
                # cv2.waitKey(0)

                #print self.theta/PI*180
                # next_x = int(round(self.center_x + self.speed*cos(self.theta)))
                # next_y = int(round(self.center_y + self.speed*sin(self.theta)))

                #print (next_y, next_x)

                # flag = 1
                # while flag:
                #     flag = 0
                #     # print theta
                #     if next_x < 0:
                #         self.theta = PI-self.theta
                #         flag = 1
                #     elif next_x >= width:
                #         self.theta = PI-self.theta
                #         flag = 1
                #     elif next_y < 0:
                #         self.theta = -self.theta
                #         flag = 1
                #     elif next_y >= height:
                #         self.theta = -self.theta
                #         flag = 1
                #     if flag:
                #         next_x = int(round(next_x + self.speed*cos(self.theta)))
                #         next_y = int(round(next_y + self.speed*sin(self.theta)))
        print self.theta*180/PI
        return hit_flag, (hit_x, hit_y), self.theta
        # print self.theta*180/PI            
        # self.center_x = next_x
        # self.center_y = next_y

        # return image
        # show_img = img.copy()
        # cv2.circle(show_img, (self.center_x,self.center_y), 5, 150, -1)
        # return show_img
        # return point
        # return (self.center_y, self.center_x), self.theta 



# import numpy as np
# import cv2
# from math import *
# from camera_setup import find_brightest_point

# PI = 3.1415926
# X_U = 234
# X_L = 126
# Y_U = 161
# Y_L = 89


# class Ball(object):
#     def __init__(self, init_x, init_y, theta, speed):
#         # initialize a point
#         self.center_x = init_x
#         self.center_y = init_y
#         self.theta = float(theta)/180*PI
#         self.speed = speed

#     def updateBall(self, img, gray):

#         height, width = img.shape
#         #img = cv2.medianBlur(img,5)

#         flag = 1
#         while flag:
#             flag = 0
#             # print theta
#             # last_x = self.center_x
#             # last_y = self.center_y

#             # brighest = find_brightest_point(gray, 1)
#             # self.center_x = brighest[1][0][0]
#             # self.center_y = brighest[1][0][1]
#             # if 

#             next_x = int(round(self.center_x + self.speed*cos(self.theta)))
#             next_y = int(round(self.center_y + self.speed*sin(self.theta)))
#             if next_x < X_L:
#                 self.theta = PI-self.theta
#                 flag = 1
#             elif next_x >= X_U:
#                 self.theta = PI-self.theta
#                 flag = 1
#             elif next_y < Y_L:
#                 self.theta = -self.theta
#                 flag = 1
#             elif next_y >= Y_U:
#                 self.theta = -self.theta
#                 flag = 1
#         # print (next_y, next_x)
#         # print theta/PI*180
#         # check if next position hits foreground
#         # if img[next_y, next_x] > 0:
#         hit_flag = 0
#         dx = -cos(self.theta)
#         dy = -sin(self.theta)
#         for i in xrange(int(self.speed/5)):
#             dx_i = int(round(i*dx))
#             dy_i = int(round(i*dy))
#             if img[next_y + dy_i, next_x + dx_i] > 0:
#                 print i
#                 hit_flag = 1
#             if hit_flag and img[next_y + dy_i, next_x + dx_i] == 0:
#                 break
#         if hit_flag:
#             hit_x = next_x + dx_i
#             hit_y = next_y + dy_i

#             RANGE = 10
#             window_hit = img[max(hit_y-RANGE,0):min(hit_y+RANGE,height), max(hit_x-RANGE,0):min(hit_x+RANGE,width)]
#             # hi_img = np.zeros((2*RANGE+1,2*RANGE+1),np.uint8)
#             edges = cv2.Canny(window_hit,50,150,apertureSize = 3)
#             # cv2.imshow('new', edges)
#             # cv2.waitKey(0)

#             lines = cv2.HoughLinesP(edges,1,np.pi/180,10)
#             number  = 0
#             alpha = 0
#             try:
#                 for x1,y1,x2,y2 in lines[0]:
#                     # cv2.line(hi_img,(x1,y1),(x2,y2),255,2)
#                     # print (x1,y1,x2,y2)
#                     alpha = (float(alpha*number) + float(atan2(y2 - y1, x2 - x1)))/float((number+1))
#                     number = number + 1
#                     #print number
#             except:
#                 pass

#             if number>0:
#                 # print alpha/PI*180
#                 self.theta = 2*alpha - self.theta

#                 # cv2.imshow('hi_img', hi_img)
#                 # cv2.waitKey(0)

#                 #print self.theta/PI*180
#                 # next_x = int(round(self.center_x + self.speed*cos(self.theta)))
#                 # next_y = int(round(self.center_y + self.speed*sin(self.theta)))
#                 next_x = hit_x
#                 next_y = hit_y
#                 #print (next_y, next_x)

#                 flag = 1
#                 while flag:
#                     flag = 0
#                     # print theta
#                     if next_x < X_L:
#                         self.theta = PI-self.theta
#                         flag = 1
#                     elif next_x >= X_U:
#                         self.theta = PI-self.theta
#                         flag = 1
#                     elif next_y < Y_L:
#                         self.theta = -self.theta
#                         flag = 1
#                     elif next_y >= Y_U:
#                         self.theta = -self.theta
#                         flag = 1
#                     if flag:
#                         next_x = int(round(next_x + self.speed*cos(self.theta)))
#                         next_y = int(round(next_y + self.speed*sin(self.theta)))


#         # print self.theta*180/PI            
#         self.center_x = next_x
#         self.center_y = next_y

#         # return image
#         show_img = img.copy()
#         cv2.circle(show_img, (self.center_x,self.center_y), 5, 150, -1)
#         return show_img
#         # return point
#         # return (self.center_y, self.center_x), self.theta 



# import numpy as np
# import cv2
# from math import *
# from camera_setup import find_brightest_point

# PI = 3.1415926
# X_U = 234
# X_L = 126
# Y_U = 161
# Y_L = 89


# class Ball(object):
#     def __init__(self, init_x, init_y, theta, speed):
#         # initialize a point
#         self.center_x = init_x
#         self.center_y = init_y
#         self.theta = float(theta)/180*PI
#         self.speed = speed

#     def updateBall(self, img, gray):

#         height, width = img.shape
#         #img = cv2.medianBlur(img,5)

#         flag = 1
#         while flag:
#             flag = 0
#             # print theta
#             # last_x = self.center_x
#             # last_y = self.center_y

#             # brighest = find_brightest_point(gray, 1)
#             # self.center_x = brighest[1][0][0]
#             # self.center_y = brighest[1][0][1]
#             # if 

#             next_x = int(round(self.center_x + self.speed*cos(self.theta)))
#             next_y = int(round(self.center_y + self.speed*sin(self.theta)))
#             if next_x < X_L:
#                 self.theta = PI-self.theta
#                 flag = 1
#             elif next_x >= X_U:
#                 self.theta = PI-self.theta
#                 flag = 1
#             elif next_y < Y_L:
#                 self.theta = -self.theta
#                 flag = 1
#             elif next_y >= Y_U:
#                 self.theta = -self.theta
#                 flag = 1
#         # print (next_y, next_x)
#         # print theta/PI*180
#         # check if next position hits foreground
#         if img[next_y, next_x] > 0:
#             dx = -cos(self.theta)
#             dy = -sin(self.theta)
#             for i in xrange(int(self.speed/5)):
#                 dx_i = int(round(i*dx))
#                 dy_i = int(round(i*dy))
#                 if img[next_y + dy_i, next_x + dx_i] == 0:
#                     break
#             hit_x = next_x + dx_i
#             hit_y = next_y + dy_i

#             RANGE = 10
#             window_hit = img[max(hit_y-RANGE,0):min(hit_y+RANGE,height), max(hit_x-RANGE,0):min(hit_x+RANGE,width)]
#             # hi_img = np.zeros((2*RANGE+1,2*RANGE+1),np.uint8)
#             edges = cv2.Canny(window_hit,50,150,apertureSize = 3)
#             # cv2.imshow('new', edges)
#             # cv2.waitKey(0)

#             lines = cv2.HoughLinesP(edges,1,np.pi/180,10)
#             number  = 0
#             alpha = 0
#             try:
#                 for x1,y1,x2,y2 in lines[0]:
#                     # cv2.line(hi_img,(x1,y1),(x2,y2),255,2)
#                     # print (x1,y1,x2,y2)
#                     alpha = (float(alpha*number) + float(atan2(y2 - y1, x2 - x1)))/float((number+1))
#                     number = number + 1
#                     #print number
#             except:
#                 pass

#             if number>0:
#                 # print alpha/PI*180
#                 self.theta = 2*alpha - self.theta

#                 # cv2.imshow('hi_img', hi_img)
#                 # cv2.waitKey(0)

#                 #print self.theta/PI*180
#                 # next_x = int(round(self.center_x + self.speed*cos(self.theta)))
#                 # next_y = int(round(self.center_y + self.speed*sin(self.theta)))
#                 next_x = hit_x
#                 next_y = hit_y
#                 #print (next_y, next_x)

#                 flag = 1
#                 while flag:
#                     flag = 0
#                     # print theta
#                     if next_x < X_L:
#                         self.theta = PI-self.theta
#                         flag = 1
#                     elif next_x >= X_U:
#                         self.theta = PI-self.theta
#                         flag = 1
#                     elif next_y < Y_L:
#                         self.theta = -self.theta
#                         flag = 1
#                     elif next_y >= Y_U:
#                         self.theta = -self.theta
#                         flag = 1
#                     if flag:
#                         next_x = int(round(next_x + self.speed*cos(self.theta)))
#                         next_y = int(round(next_y + self.speed*sin(self.theta)))


#         print self.theta*180/PI            
#         self.center_x = next_x
#         self.center_y = next_y

#         # return image
#         show_img = img.copy()
#         cv2.circle(show_img, (self.center_x,self.center_y), 5, 150, -1)
#         return show_img
#         # return point
#         # return (self.center_y, self.center_x), self.theta 
