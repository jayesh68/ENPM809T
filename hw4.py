# import the necessary packages

from picamera.array import PiRGBArray
from picamera import PiCamera
import time as t
import cv2
import numpy as np
import datetime
import math
# initialize the Raspberry Pi camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 25
rawCapture = PiRGBArray(camera, size=(640,480))
# allow the camera to warmup
t.sleep(0.1)
# keep looping
f =open('hw3data.txt','w')
# define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('hw82.avi', fourcc, 10, (640, 480))
a=[]
b=[]

count1=0
count2=0
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=False):
    # grab the current frame
    start=datetime.datetime.now()
    print('start',start)
    img = frame.array
    # show the frame to our screen
    cv2.imshow("Frame", img)
    
    rows,cols,chn=img.shape

    blank_img=np.zeros(shape=[rows,cols,chn],dtype=np.uint8)

    cv2.imshow('blank image',blank_img)

    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    cv2.imshow('hsv',hsv)
    '''	
    lower_green = np.array([58,138,138])
    upper_green = np.array([82,255,255])
    
    lower_green = np.array([0,145,224])
    upper_green = np.array([90,255,255])

    lower_green = np.array([0,87,215])
    upper_green = np.array([255,255,255])
    
    lower_green = np.array([22,122,104])
    upper_green = np.array([88,255,255])
    '''
    lower_green = np.array([0,95,126])
    upper_green = np.array([78,255,255])
    
    mask = cv2.inRange(hsv,lower_green,upper_green) 	
    cv2.imshow('mask',mask)
    '''
    mask=cv2.erode(mask,None,iterations=3)
    mask=cv2.dilate(mask,None,iterations=3) 
    cv2.imshow('mask',mask)
    '''
    mask_stack=np.stack((mask,)*3,axis=-1)
    gblur=cv2.GaussianBlur(mask_stack,(5,5),0)
    gblur1=cv2.cvtColor(gblur,cv2.COLOR_BGR2GRAY)
    cv2.imshow('gblur',gblur1)
         
    contours,hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
   

    cv2.drawContours(img,contours,-1,(36,255,12),2)
    corners=cv2.goodFeaturesToTrack(gblur1,7,0.04,10)
    if corners is None:
    	rawCapture.truncate(0)
    	print('nothing')
    		#continue
    
    else:
    	corners=np.int0(corners)
    	for i in corners:
    		x,y=i.ravel()

    		cv2.circle(img,(x,y),3,255,-1)

    	xmax,ymax=(np.max(corners,axis=0)).ravel()
    	xmin,ymin=(np.min(corners,axis=0)).ravel()
    	print('xmax',xmax,'ymax',ymax,'xmin',xmin,'ymin',ymin)
    	print('corners',corners[:,0,0])
    	print('corners1',corners[:,0,1])
    	font=cv2.FONT_HERSHEY_SIMPLEX
    	if(abs(xmax-xmin)>abs(ymax-ymin)):
    		a=corners[:,0,0]
    		count=0
    		for i in range(0,len(a)):
    			if math.isclose(a[i],xmax,abs_tol=5):
    				count+=1 
    		if count>=2:
    			cv2.putText(img,'left',(350,300),font,1,(255,155,55),2,cv2.LINE_AA)
    			#cv2.putText(img,str(count),(450,300),font,1,(255,155,55),2,cv2.LINE_AA)
    			print('left')
    		else:
    			cv2.putText(img,'right',(350,300),font,1,(255,155,55),2,cv2.LINE_AA)
    			#cv2.putText(img,str(count),(450,300),font,1,(255,155,55),2,cv2.LINE_AA)
    			print('right')
					
    	else:
    		b=corners[:,0,1]
    		count1=0
    		for i in range(0,len(b)):
    			if math.isclose(b[i],ymax,abs_tol=5):
    				count1+=1
    		if count1>=2:
    			cv2.putText(img,'up',(350,300),font,1,(255,155,55),2,cv2.LINE_AA)
    			#cv2.putText(img,str(count1),(450,300),font,1,(255,155,55),2,cv2.LINE_AA)    			    			
    			print('up')
    		else:
    			cv2.putText(img,'down',(350,300),font,1,(255,155,55),2,cv2.LINE_AA)
    			#cv2.putText(img,str(count1),(450,300),font,1,(255,155,55),2,cv2.LINE_AA)	
    			print('down')  

    cv2.imshow('Image1',img)
    
    key = cv2.waitKey(1) & 0xFF
    
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    count2+=1
    # press the 'q' key to stop the video stream
    if key == ord("q"):
        break
    end=datetime.datetime.now()
    print('end',end)
    
    now=end-start
    print(now)
    outstring=str(count2)+' '+str(now.total_seconds())+'\n'
    f.write(outstring)
    print(now.total_seconds())
    # write frame to video file
    out.write(img)
    if count2 == 200:
    	break
f.close()
