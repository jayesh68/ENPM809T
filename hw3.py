# import the necessary packages

from picamera.array import PiRGBArray
from picamera import PiCamera
import time as t
import cv2
import numpy as np
import datetime
import time

# initialize the Raspberry Pi camera
camera = cv2.VideoCapture(0)
#camera.resolution = (640, 480)
#camera.framerate = 25
#rawCapture = PiRGBArray(camera, size=(640,480))
# allow the camera to warmup
#t.sleep(0.1)
# keep looping
f =open('hw3data.txt','w')
# define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('hw35.avi', fourcc, 10, (640, 480))
count=0
#for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=False):
    # grab the current frame
while True:
    ret, frame=camera.read()
    frame=cv2.flip(frame,-1) 
    start=datetime.datetime.now()
    print('start',start)
    image = frame.copy()
    # show the frame to our screen
    cv2.imshow("Frame", image)
    
    rows,cols,chn=image.shape

    blank_img=np.zeros(shape=[rows,cols,chn],dtype=np.uint8)

    cv2.imshow('Image',image)
    cv2.imshow('blank image',blank_img)

    hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    cv2.imshow('hsv',hsv)
    '''
    lower_green = np.array([100,186,32])
    upper_green = np.array([127,255,110])

    lower_green = np.array([83,95,93])
    upper_green = np.array([150,255,255])
    '''
    lower_green = np.array([63,76,43])
    upper_green = np.array([88,216,121])
   
    blank_img = cv2.inRange(hsv,lower_green,upper_green)

    blank_img=cv2.erode(blank_img,None,iterations=2)
    blank_img=cv2.dilate(blank_img,None,iterations=2)
    
    cv2.imshow('mask',blank_img)

    contours,hierarchy = cv2.findContours(blank_img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
    cx1=[]
    cy1=[]
    rad=[]
        #print('contours',len(contours))

    if len(contours) > 0:
        c=max(contours,key=cv2.contourArea)
        ((x,y),radius)=cv2.minEnclosingCircle(c)
        M=cv2.moments(c)
        area=cv2.contourArea(c)
        if M["m00"]!=0:
            #print('moments')
            cx=int(M["m10"]/M["m00"])
            cy=int(M["m01"]/M["m00"])
            cx1.append(cx)
            cy1.append(cy)
        else:
            cx,cy=0,0
            cx1.append(cx)
            cy1.append(cy)
        center=(cx,cy)
        print('radius','area',radius,area)
        #if radius > 20:
        #    print('rad',radius)
        cv2.circle(image,center,int(radius),((0,255,255)),2)
    #else:
        #continue
    #rad.append(radius)
    #center=(cx,cy)
    #cv2.circle(image,center,int(radius),((0,255,255)),2) 
    #radius_fin=max(rad)
    #centre_x=int((min(cx1)+max(cx1))/2)
    #centre_y=int((min(cy1)+max(cy1))/2)
    #centre_x=max(cx1)
    #centre_y=max(cy1)
    #cv2.circle(image,(int(x),int(y)),1,(255,25,255),1)
    #cv2.circle(image,(int(centre_x),int(centre_y)),int(radius_fin),(255,25,255),1)
    cv2.imshow('Image1',image)
    
    key = cv2.waitKey(1) & 0xFF
    
    # clear the stream in preparation for the next frame
    #rawCapture.truncate(0)
    # press the 'q' key to stop the video stream
    count+=1
    if key == ord("q"):
        break
    end=datetime.datetime.now()
    print('end',end)
    
    now=end-start
    print(now)
    outstring=str(count)+' '+str(now.total_seconds())+'\n'
    f.write(outstring)
    print(now.total_seconds())
    # write frame to video file
    out.write(image)
    if count == 1024:
    	break
  
f.close()
