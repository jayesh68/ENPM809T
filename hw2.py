import numpy as np
from matplotlib import pyplot as plt
import cv2
import math
import imutils

img1 = cv2.imread('hw3.jpg')

rows,cols,chn=img1.shape

img1=cv2.resize(img1,(int(rows/4),int(cols/4)))

img = img1.copy()

rows,cols,chn=img.shape
print(rows,cols,chn)
rows,cols,chn=img.shape

#img=cv2.resize(img,(int(rows/4),int(cols/4)))

#cv2.imshow('Image',img)
#cv2.imshow('blank image',blank_img)

hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
#cv2.imshow('hsv',hsv)

lower_green = np.array([56,130,0])
#lower_green = np.array([36,0,0])
upper_green = np.array([86,255,255])

blank_img = cv2.inRange(hsv,lower_green,upper_green)
blank_stack=np.stack((blank_img,)*3,axis=-1)
rows,cols,chn=blank_stack.shape
print(rows,cols,chn,'blank_stack')
img_show = np.hstack((img1,hsv,blank_stack))
cv2.imshow('Stacked images',img_show)

contours,hierarchy = cv2.findContours(blank_img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
#print('contours',contours)
for i in range(0,len(contours)):
    ((x,y),radius)=cv2.minEnclosingCircle(contours[i])
    M=cv2.moments(contours[i])
    if M["m00"]!=0:
        cx=int(M["m10"]/M["m00"])
        cy=int(M["m01"]/M["m00"])
    else:
        cx,cy=0,0
    center=(cx,cy)
    #cv2.circle(img,(int(x),int(y)),int(radius),(255,25,255),1)
    cv2.circle(img,center,int(radius),(0,255,255),2)     

img_show1 = np.hstack((img1,hsv,blank_stack,img))
cv2.imshow('Stacked images final',img_show1)

cv2.waitKey(0)

#cv2.destoryAllWindows()
