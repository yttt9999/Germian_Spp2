import cv2
from numpy import linalg as LA
import math

red = (0,0,255)
blue = (255,255,255)
image = cv2.imread('autocali_1.jpg')
image1 = image
cv2.line(image1,(100,100),(200,200),red,2)
cv2.circle(image1,(100,100),2,blue)
cv2.circle(image1,(110,110),2,blue)
cv2.circle(image1,(120,121),2,blue)
cv2.circle(image1,(130,131),2,blue)
cv2.circle(image1,(140,141),2,blue)
cv2.circle(image1,(150,151),2,blue)
cv2.circle(image1,(160,161),2,blue)
cv2.circle(image1,(170,172),2,blue)
cv2.circle(image1,(180,178),2,blue)
cv2.circle(image1,(190,193),2,blue)
cv2.circle(image1,(200,198),2,blue)
while True:
    cv2.imshow("test",image1)
    if cv2.waitKey(1) & 0xFF == ord("q"):
            break
cv2.imwrite('inte.jpg',image1)
cv2.destroyAllWindows()
MyFlycam.stop_capture()
MyFlycam.disconnect()
    



