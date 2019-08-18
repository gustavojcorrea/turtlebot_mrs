#!/usr/bin/env python

import numpy as np
import cv2
import cv2 as cv
import time
import rospy
from std_msgs.msg import UInt16, Int16

rospy.init_node('Colored_Circle_Tracker')

# Capture object to grab camera images
cap = cv2.VideoCapture(0)

last_time = time.time()

# Starting with 100's to prevent error while masking
h,s,v = 0, 35, 0

def nothing(x):
	pass
cv2.namedWindow('result')

# Creating track bar
cv2.createTrackbar('h', 'result',0,179, nothing)
cv2.createTrackbar('s', 'result',0,255, nothing)
cv2.createTrackbar('v', 'result',0,255, nothing)


cv2.createTrackbar('h1', 'result',0,179, nothing)
cv2.createTrackbar('s1', 'result',35,255, nothing)
cv2.createTrackbar('v1', 'result',0,255, nothing)

cv2.setTrackbarPos('h','result',0)
cv2.setTrackbarPos('s','result',0)
cv2.setTrackbarPos('v','result',0)
cv2.setTrackbarPos('h1','result',014)
cv2.setTrackbarPos('s1','result',255)
cv2.setTrackbarPos('v1','result',255)

# Variable to be published
xaxis = UInt16()
xaxis.data = 90
yaxis = UInt16()
yaxis.data = 170

# Frame size and offset for detection
frame_width = 400
frame_height = 300
offset = 25  # To create target box of 25 x 25 pixels
# X-axis servo limits
xservo_max = 170
xservo_min = 10
# Y-axis servo limits
yservo_max = 170
yservo_min = 0

print ('box: {}'.format( frame_width/2 + offset))

def mapxpos_toservo(xpos_obj):
	if (xpos_obj > 0):
		if (xpos_obj > frame_width/2 + offset):
			#rotate right
			if (xaxis.data > xservo_min + 5):
                        	xaxis.data -= 3
		elif (xpos_obj < frame_width/2 - offset):
			#rotate left
			if (xaxis.data < xservo_max - 5):
				xaxis.data += 3

def mapypos_toservo(ypos_obj):
	if (ypos_obj > 0):
		if (ypos_obj > frame_height/2 + offset):
			# rotate down
			if (yaxis.data < yservo_max - 5):
				yaxis.data += 3
		elif (ypos_obj < frame_height/2 - offset):
			# rotate up
			if (yaxis.data > yservo_min + 5):
				yaxis.data -= 3


# Object to publish to a particular topic parameters: !

xaxis_pub = rospy.Publisher('Face_xaxis',Int16, queue_size=1)
yaxis_pub = rospy.Publisher('Face_yaxis',Int16, queue_size=1)
# direction_pub =

# Rate at which the the circle detector node publishes to topics
rate = rospy.Rate(20) #20 Hz

# Execute node as long as it's not shutdown
while not rospy.is_shutdown():
#while True:
	ret,frame = cap.read()
	frame = cv2.resize(frame,(frame_width,frame_height))
#	frame = cv2.medianBlur(frame,5)

	#Perform color based image thresholding using HSV color space
	frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	# get info from track bar and appy to result
	h = cv2.getTrackbarPos('h','result')
	s = cv2.getTrackbarPos('s','result')
	v = cv2.getTrackbarPos('v','result')

	h1 = cv2.getTrackbarPos('h1','result')
	s1 = cv2.getTrackbarPos('s1','result')
	v1 = cv2.getTrackbarPos('v1','result')

	lower_orange = np.array([h,s,v])
	upper_orange = np.array([h1,s1,v1])

	thresh_img = cv2.inRange(frameHSV, lower_orange, upper_orange)

	result = cv2.bitwise_and(frame, frame, mask = thresh_img)

	result = cv2.cvtColor(result, cv2.COLOR_HSV2BGR)
	result = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)

	circles = cv2.HoughCircles(result,cv.HOUGH_GRADIENT,
				   1,300,param1=20,param2=30,
				   minRadius=50 ,maxRadius=100)

	# print('Frame took {} seconds'.format(time.time()-last_time))
    #     last_time = time.time()\

	x = 0
	y = 0

	cv2.line(frame,(133,0),(133,200),(255,0,0),5)
	cv2.line(frame,(266,0),(266,200),(255,0,0),5)
	cv2.line(frame,(0,200),(400,200),(255,0,0),5)
	if circles is not  None:
		circles = np.uint16(np.around(circles))
		for i in circles[0,:]:
			# Draw Outer circle
			# parameters: (frame, x, y, radius, color, thickness)
			cv2.circle(frame,(i[0],i[1]),i[2],(0,255,0),2)
			# Draw Centroid
			cv2.circle(frame,(i[0],i[1]),2,(0,0,255),3)
			x = i[0]
			y = i[1]
			text = '(' + str(i[0]) + ', ' + str(i[1]) + ')'
			cv2.putText(frame,text,(i[0],i[1]),cv2.FONT_ITALIC,1, 255)

	# publish mapping
	print('Xpos: {}, Ypos: {} '.format(x, y))
	mapxpos_toservo(x)
	mapypos_toservo(y)
	# xaxis_pub.publish(xaxis)
	# yaxis_pub.publish(yaxis)

	xaxis_pub.publish(x)
	yaxis_pub.publish(y)

	cv2.imshow('Thresh img', result)
	cv2.imshow('Detected circles', frame)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		cv2.destroyAllWindows()
		break
