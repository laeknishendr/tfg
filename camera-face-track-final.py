#!/bin/sh
from picamera.array import PiRGBArray
from picamera import PiCamera
from arduino2 import Arduino
import time 
import cv2
import numpy as np
import serial
from auxmethods import printData 

#arduino com parameters
aMotor =int(6)				# pin number for motor A
bMotor = int(9)				# pin number for motor B
aStartAngle = 60	
bStartAngle = 120

board = Arduino(9600,port='/dev/ttyACM0')
board.Servos.attach(aMotor)
board.Servos.write(aMotor, aStartAngle)
board.Servos.attach(bMotor)
board.Servos.write(bMotor,bStartAngle)

#camera parameters 
camera = PiCamera()
camera.resolution = (320,240)
camera.framerate=60
rawCapture = PiRGBArray(camera, size = (320,240))
time.sleep(0.1)

#face recognition parameters
faceCascade = cv2.CascadeClassifier("/home/pi/opencv-3.1.0/data/haarcascades/haarcascade_frontalface_default.xml")
meanFaces = []
ImageCenterX = 160
ImageCenterY = 120

#sensibility parameters
imageWidth = 40
imageHeight = 40
precision = 10			# set tolerance in pixels for face position tracking
patrolMode = True
patrolStartAngle = 90

#main face detection&shit loop
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	image = frame.array
	gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
	faces = faceCascade.detectMultiScale(gray,scaleFactor=1.1,minNeighbors=5,minSize=(30, 30),flags = cv2.CASCADE_SCALE_IMAGE)
	
#servo moving
	if not(faces==()):
		x = faces[0][0]
		y = faces[0][1]
		w = faces[0][2]
		h = faces[0][3]
	#B motor handling
		#face is in the left
		if x+int(w/2) < ImageCenterX-precision: 
			print('moving to the left!')
			bStartAngle = bStartAngle + 1
			board.Servos.write(bMotor,bStartAngle)
			board.canSend([201,aStartAngle, bStartAngle])
			lastKnownMovementX = 1		
		#face is in the right
		elif x+int(w/2) > ImageCenterX+precision: 
			print('moving to the right!')
			bStartAngle = bStartAngle - 1
			board.Servos.write(bMotor,bStartAngle)
			board.canSend([201,aStartAngle, bStartAngle])
			lastKnownMovementX=2
	#A motor handling
		#face is down
		if y+int(y/2) < ImageCenterY-precision:
			aStartAngle = aStartAngle - 1
			board.Servos.write(aMotor,aStartAngle)
			board.canSend([201,aStartAngle, bStartAngle])
			lastKnownMovementY = 1
		#face is up 
		elif y+int(y/2) > ImageCenterY+precision:
			aStartAngle = aStartAngle + 1
			board.Servos.write(aMotor,aStartAngle)
			board.canSend([201,aStartAngle, bStartAngle])
			lastKnownMovementY=2

#patrol mode execution	
	elif (faces == ()) & patrolMode:
		patrolStartAngle = bStartAngle
		if patrolStartAngle < 150: 
			patrolStartAngle = patrolStartAngle + 1 
			board.Servos.write(bMotor, patrolStartAngle)
		if patrolStartAngle == 150: 
			patrolStartAngle = 30
			board.Servos.write(bMotor,patrolStartAngle)	
		       
	for(x, y, w, h) in faces: 
		width = int(x+(w/2))
		heigth = int(y+(h/2))
		cv2.line(image,(0,y+int(h/2)), (320, y+int(h/2)), (0,0,255), 1)		#horizontal line
		cv2.line(image, (x+int(w/2), 0), (x+int(w/2),240), ( 0,0,255), 1)	#vertical line
		cv2.rectangle(image, (x, y), (x+w,y+h), (0, 255, 0), 2) 			#rectangle 
		printData(image,x,y,w,h)

	
#	cv2.imshow("image", image)
	key = cv2.waitKey(1)&0xFF
	rawCapture.truncate(0)
	
        if key == ord("q"):
                break



