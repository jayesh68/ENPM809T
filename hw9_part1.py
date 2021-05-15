# import the necessary packages

from picamera.array import PiRGBArray
from picamera import PiCamera
import RPi.GPIO as gpio
import time as t
import cv2
import numpy as np
import datetime
import serial
import time
import math
# initialize the Raspberry Pi camera
#camera = cv2.VideoCapture(0)
camera = PiCamera()
camera.rotation=180
camera.resolution = (640, 480)
camera.framerate = 2

rawCapture = PiRGBArray(camera, size=(640,480))
# allow the camera to warmup
t.sleep(0.1)
ser=serial.Serial('/dev/ttyUSB0',9600)
gpio.setmode(gpio.BOARD)
gpio.setup(31,gpio.OUT)
gpio.setup(33,gpio.OUT)
gpio.setup(35,gpio.OUT)
gpio.setup(37,gpio.OUT)
gpio.setup(36,gpio.OUT)
pwm=gpio.PWM(36,50)
pwm.start(5)
gpio.setup(7,gpio.IN,pull_up_down=gpio.PUD_UP)
gpio.setup(12,gpio.IN,pull_up_down=gpio.PUD_UP)
curr_ang=0
# define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('hw101.avi', fourcc, 10, (640, 480))
count=0

def gameover():
	#Set all pins low
	gpio.output(31,False)
	gpio.output(33,False)
	gpio.output(35,False)
	gpio.output(37,False)
    
def pivot_left():
	print('left pivoting')
	counterBR=np.uint64(0)   
	counterFL=np.uint64(0)
	buttonBR=int(0)
	buttonFL=int(0)

	pwm1=gpio.PWM(33,50) #left
	pwm2=gpio.PWM(37,50) #right
	val=22
	pwm1.start(85)
	pwm2.start(85)
	time.sleep(0.1)

	#print("Pivot Left","counterBR=",counterBR,"counterFL=",counterFL,"BR state:",gpio.input(12),"FL state:" ,gpio.input(7))

	if int(gpio.input(12))!=int(buttonBR):
		buttonBR=int(gpio.input(12))
		counterBR+=1
		#print(counterBR)
        #pwm2.stop()

	if int(gpio.input(7))!=int(buttonFL):
		buttonFL=int(gpio.input(7))
		counterFL+=1
		#print(counterFL)
        #pwm1.stop()
	
def pivot_right():
	print('pivoting right')
	counterBR=np.uint64(0)
	counterFL=np.uint64(0)

	buttonBR=int(0)
	buttonFL=int(0)

	pwm1=gpio.PWM(31,50) #left
	pwm2=gpio.PWM(35,50) #right

	pwm1.start(85)
	pwm2.start(85)
	time.sleep(0.1)

	#print("Pivot Left","counterBR=",counterBR,"counterFL=",counterFL,"BR state:",gpio.input(12),"FL state:" ,gpio.input(7))

	if int(gpio.input(12))!=int(buttonBR):
		buttonBR=int(gpio.input(12))
		counterBR+=1
		#print(counterBR)
        #pwm2.stop()
	if int(gpio.input(7))!=int(buttonFL):
		buttonFL=int(gpio.input(7))
		counterFL+=1
		#print(counterFL)
        #pwm1.stop()
    
def imu_orient(angle):
    count=0
    count1=0
    new_ang=0
    count2=0
    #curr_ang1=359
    #curr_ang2=359
    print('IMU Orient')
    while True:
        if(ser.in_waiting > 0):
            line=ser.readline()
            count+=1
	    
            if count > 10:
            #read serial stream
                line=ser.readline()
                line=line.rstrip().lstrip()
                line=str(line)
                line=line.strip("'")
                line=line.strip("b'")
                curr_ang=float(line)
                print('init angle',curr_ang)
                new_ang=curr_ang
                print('angle track',new_ang)

                if -2<=angle<=2:
                    break
		    
                if angle>0:
                    print('angle>0')
                    while ((curr_ang<=new_ang<=(curr_ang+angle))):
                        print(curr_ang)
                        print(angle)			 
                        #print(curr_ang+((curr_ang-angle)%360)) 						    
                        print('in first while')   
                        '''
                        if math.isclose(new_ang,360,abs_tol=2) or  math.isclose(new_ang,0,abs_tol=2):
                                z=abs(360-curr_ang)
                                diff=angle-z
                                print(diff)
                                break
                        '''	
                        pivot_right()
                        line=ser.readline()
                        line=line.rstrip().lstrip()
                        line=str(line)
                        line=line.strip("'")
                        line=line.strip("b'")
                        new_ang=float(line)
                        print('new_ang',new_ang)
                        #print('close',math.isclose(new_ang,0,abs_tol=2))
                    ''' 	
                    line=ser.readline()
                    line=line.rstrip().lstrip()
                    line=str(line)
                    line=line.strip("'")
                    line=line.strip("b'")
                    new_ang=float(line)
		    '''
                    if curr_ang + angle>= 360:
                        print('360 here')
                        new_ang=0
                        angle_covered=360-curr_ang	
                        angle_left=angle-angle_covered		    
                        while 0<=new_ang<=angle_left:
                             print('in second while')				
                             pivot_right()
                             line=ser.readline()
                             line=line.rstrip().lstrip()
                             line=str(line)
                             line=line.strip("'")
                             line=line.strip("b'")
                             new_ang=float(line)
                             print('new_ang',new_ang)
			    
                        #print('close',math.isclose(new_ang,0,abs_tol=2))
                        '''
                        if math.isclose(new_ang,00,abs_tol=2):
                              curr_ang1=new_ang
                        else:
                              curr_ang1=0
                              print('curr_ang1',curr_ang)
                        '''
                if angle<0:
                    print('angle<0')
                    print('diff',abs(curr_ang-new_ang),abs(curr_ang+angle))

                    while (curr_ang+angle)<=new_ang<=curr_ang:
                        '''			    
                        if math.isclose(new_ang,00,abs_tol=2) or math.isclose(new_ang,359,abs_tol=2):
                             z=abs(0-curr_ang)
                             diff1=abs(-(angle)-z)
                             print('diff1',diff1)
                             print('break')
                             break
                        '''			     
                        pivot_left()
                        line=ser.readline()
                        line=line.rstrip().lstrip()
                        line=str(line)
                        line=line.strip("'")
                        line=line.strip("b'")
                        new_ang=float(line)
                        print('new_ang',new_ang)
                        #print('cur_ang',curr_ang1)
                        #print('close',math.isclose(new_ang,360,abs_tol=2))

				     												
                    line=ser.readline()
                    line=line.rstrip().lstrip()
                    line=str(line)
                    line=line.strip("'")
                    line=line.strip("b'")
                    new_ang=float(line)
		    
                    if curr_ang + angle<= 0: 
                        new_ang=360
                        angle_covered=curr_ang	
                        angle_left=angle-angle_covered			
                        print('angle<360')		    
                        while 360-angle_left<=new_ang<=360:
                             pivot_left()
                             line=ser.readline()
                             line=line.rstrip().lstrip()
                             line=str(line)
                             line=line.strip("'")
                             line=line.strip("b'")
                             new_ang=float(line)
                             print('new_ang',new_ang)


                break
		
count2=0
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=False):
        count2+=1
        print(count2)
        #ret,frame=camera.read() 
        #t.sleep(3)
        #frame=cv2.flip(frame,-1) 	
        # grab the current frame
        #frame=np.uint8(frame)
        #frame=cv2.flip(frame,-1)
        #start=datetime.datetime.now()
        #print('start',start)
        #image = frame.copy()
        image = frame.array
        # show the frame to our screen
        #cv2.imshow("Frame", image)
    
        rows,cols,chn=image.shape
    
        blank_img=np.zeros(shape=[rows,cols,chn],dtype=np.uint8)

        #cv2.imshow('Image',image)
        #cv2.imshow('blank image',blank_img)

        hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        #cv2.imshow('hsv',hsv)
        '''
        lower_green = np.array([31,122,15])
        upper_green = np.array([59,255,120])
        
        lower_green = np.array([36,111,11])
        upper_green = np.array([55,255,217])
	'''
        lower_green = np.array([32,80,16])
        upper_green = np.array([60,173,186])
	
        blank_img = cv2.inRange(hsv,lower_green,upper_green)
        blank_img=cv2.erode(blank_img,None,iterations=3)
        blank_img=cv2.dilate(blank_img,None,iterations=3) 
        cv2.imshow('mask',blank_img)

        contours,hierarchy = cv2.findContours(blank_img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cx1=[]
        cy1=[]
        rad=[]
        #print('contours',len(contours))
        '''  
        for i in range(0,len(contours)):
            ((x,y),radius)=cv2.minEnclosingCircle(contours[i])
            M=cv2.moments(contours[i])
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
            rad.append(radius)
            center=(cx,cy)
            #cv2.circle(image,center,int(radius),((0,255,255)),2)
        '''	    

        if len(contours)>0:
                c=max(contours,key=cv2.contourArea)
                ((x,y),radius)=cv2.minEnclosingCircle(c)
                M=cv2.moments(c)
                if M["m00"]!=0:
                   #print('moments')
                   cx=int(M["m10"]/M["m00"])
                   cy=int(M["m01"]/M["m00"])
                #cx1.append(cx)
                #cy1.append(cy)
                else:
                   cx,cy=0,0
                center=(cx,cy)
		
                if radius > 20:
                   cv2.circle(image,center,int(radius),((0,255,255)),2)
                else:
                   radius=0

        #print(cx1,cy1,rad)
        #radius_fin=max(rad)
        #centre_x=int((min(cx1)+max(cx1))/2)
        #centre_y=int((min(cy1)+max(cy1))/2)
        #centre_x=max(cx1)
        #centre_y=max(cy1)
        #cv2.circle(image,(int(centre_x),int(centre_y)),int(radius),(255,25,255),1)
	
        cv2.imshow('Image1',image)
        if radius>20:
             print('center',center[0],640-center[0])
             angle_cen=(((center[0])-320))*0.061
             print('angle',angle_cen)
             imu_orient(angle_cen)
            
        key = cv2.waitKey(1) & 0xFF
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        # clear the stream in preparation for the next frame
        #rawCapture.truncate(0)
        # press the 'q' key to stop the video stream
        #count+=1
        if key == ord("q"):
            pwm.ChangeDutyCycle(3)
            time.sleep(2)
            gameover()
            gpio.cleanup()
            pwm.stop
            break
        #end=datetime.datetime.now()
        #print('end',end)
    
        #now=end-start
        #print(now)

        #print(now.total_seconds())
        # write frame to video file
        out.write(image)
  
#capture()
