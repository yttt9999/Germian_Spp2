__author__ = 'germain'


import flycapture2 as fc2
import numpy as np
import cv2
from time import sleep
from stepper_motor_setup import *
from camera_setup import *


if __name__ == "__main__":

    MyFlycam = flycamera_init()
    MyFlycam.start_capture()

    # im = fc2.Image()
    # MyFlycam.retrieve_buffer(im)
    # gray_image = np.array(im)
    # image_origin = cv2.cvtColor(gray_image,cv2.COLOR_GRAY2BGR)
    # image_undis = undistort_image(MyFlycam)
    # cv2.imshow("before distort",image_origin)
    # cv2.imshow("after distort",image_undis)
    # cv2.waitKey(0)
    while True:
        image_undis = undistort_image(MyFlycam)
        [new_image,H] = four_pts_transormation(image_undis,reference_4_points)
        cv2.imshow("real_time",new_image)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.imwrite('origin_image.jpg',new_image)
            break
    cv2.destroyAllWindows()
    MyFlycam.stop_capture()
    MyFlycam.disconnect()