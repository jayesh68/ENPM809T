from picamera.array import PiRGBArray
from picamera import PiCamera
import RPi.GPIO as GPIO
import time
import cv2

# initialize the Raspberry Pi camera

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 25
rawCapture = PiRGBArray(camera, size=(640,480))
# allow the camera to warmup
time.sleep(0.1)

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('motgrip3.avi', fourcc, 10, (640, 480))



GPIO.setmode(GPIO.BOARD)
GPIO.setup(36,GPIO.OUT)
pwm=GPIO.PWM(36,50)
pwm.start(6)
count=0
count1=0

def init():
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(31,GPIO.OUT)
	GPIO.setup(33,GPIO.OUT)
	
	GPIO.setup(35,GPIO.OUT)
	GPIO.setup(37,GPIO.OUT)
	GPIO.setup(16, GPIO.OUT)
	GPIO.setup(18, GPIO.IN)

def gameover():
	#Set all pins low
	GPIO.output(31,False)
	GPIO.output(33,False)
	GPIO.output(35,False)
	GPIO.output(37,False)

def distance():	
	#Ensure output has no value
	GPIO.output(16, False)
	time.sleep(0.01)

	#Generate trigger pulse
	GPIO.output(16, True)
	time.sleep(0.00001)
	GPIO.output(16, False)

	#Generate echo time signal
	while GPIO.input(18) == 0:
		pulse_start = time.time()

	while GPIO.input(18) == 1:
		pulse_end = time.time()

	pulse_duration = pulse_end - pulse_start

	#Convert time to distance
	distance = pulse_duration*17150
	distance = round(distance,2)

	return distance

def forward(tf):
	init()
	#Left Wheels
	GPIO.output(31,True)
	GPIO.output(33,False)
	#Right wheels
	GPIO.output(35,False)
	GPIO.output(37,True)
	#Wait
	time.sleep(tf)
	#Send all pins low and cleanup
	gameover()
	print('Distance from object:',distance())
	#GPIO.cleanup()

def reverse(tf):
	init()
	#Left Wheels
	GPIO.output(31,False)
	GPIO.output(33,True)
	#Right wheels
	GPIO.output(35,True)
	GPIO.output(37,False)
	#Wait
	time.sleep(tf)
	#Send all pins low and cleanup
	gameover()
	print('Distance from object:',distance())
	#GPIO.cleanup()

def right(tf):
	init()
	#Left Wheels
	GPIO.output(31,True)
	GPIO.output(33,False)
	#Right wheels
	GPIO.output(35,True)
	GPIO.output(37,True)
	#Wait
	time.sleep(tf)
	#Send all pins low and cleanup
	gameover()
	print('Distance from object:',distance())
	#GPIO.cleanup()

def left(tf):
	init()
	#Left Wheels
	GPIO.output(31,False)
	GPIO.output(33,False)
	#Right wheels
	GPIO.output(35,False)
	GPIO.output(37,True)
	#Wait
	time.sleep(tf)
	#Send all pins low and cleanup
	gameover()
	print('Distance from object:',distance())
	#GPIO.cleanup()

def pivotleft(tf):
	init()
	#Left Wheels
	GPIO.output(31,False)
	GPIO.output(33,True)
	#Right wheels
	GPIO.output(35,False)
	GPIO.output(37,True)
	#Wait
	time.sleep(tf)
	#Send all pins low and cleanup
	gameover()
	#GPIO.cleanup()

def pivotright(tf):
	init()
	#Left Wheels
	GPIO.output(31,True)
	GPIO.output(33,False)
	#Right wheels
	GPIO.output(35,True)
	GPIO.output(37,False)
	#Wait
	time.sleep(tf)
	#Send all pins low and cleanupn
	gameover()
	print('Distance from object:',distance())
	#GPIO.cleanup()

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=False):

	img = frame.array
	img=cv2.flip(img,0)
	font=cv2.FONT_HERSHEY_SIMPLEX
	key = cv2.waitKey(1) & 0xFF

	if key == ord("f"):
		forward(2)
		time.sleep(2)
	
	if key == ord("r"):
		reverse(2)
		time.sleep(2)
	
	if key == ord("s"):
		right(2)
		time.sleep(2)
	
	if key == ord("l"):
		left(2)
		time.sleep(2)
	
	if key == ord("p"):
		pivotright(2)
		time.sleep(2)
	
	if key == ord("g"):
		pivotleft(2)
		pwm.ChangeDutyCycle(4.5)
		cv2.putText(img,'Duty: 4.5%',(100,20),font,1,(255,155,155),2,cv2.LINE_AA)
		time.sleep(2)
	
	if key == ord("w"):	
		pwm.ChangeDutyCycle(5)
		cv2.putText(img,'Duty: 5%',(150,20),font,1,(255,155,155),2,cv2.LINE_AA)
		time.sleep(2)	
	if key == ord("e"):
		pwm.ChangeDutyCycle(4)
		cv2.putText(img,'Duty: 4%',(150,20),font,1,(255,155,155),2,cv2.LINE_AA)
		time.sleep(2)	
	if key == ord("y"):
		pwm.ChangeDutyCycle(3)
		cv2.putText(img,'Duty: 3%',(150,20),font,1,(255,155,155),2,cv2.LINE_AA)
		time.sleep(2)	
	if key == ord("t"):
		pwm.ChangeDutyCycle(6)
		cv2.putText(img,'Duty: 6%',(150,20),font,1,(255,155,155),2,cv2.LINE_AA)
		time.sleep(2)
	cv2.imshow('Image1',img)

	
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
   	
	# press the 'q' key to stop the video stream
	if key == ord("q"):
		print('cleaning')
		pwm.stop()
		GPIO.cleanup()
		break

	# write frame to video file
	out.write(img)

