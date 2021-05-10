#!/usr/bin/env python
# coding=utf-8

import roslib
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Float64

import smach
import smach_ros
import time
import random
from exp_assignment3.msg import Num
import random
import datetime

import cv2
import sys
import time
import numpy as np
from scipy.ndimage import filters
import imutils
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist




def user_action(data):
    
    if(data=='PLAY'):
	return ('play')

    elif(data=='SLEEP'):
	return ('sleep')

    elif(data=='NORMAL'):
	return ('normal')

    elif(data=='FIND'):
	return ('find')

   




#In the normal state the robot reaches random positions. After a certain time move to the sleep behaviour.
#when it sees the ball it moves to the play behavior

class Normal(smach.State):


    def __init__(self):
        self.var='FALSE'
	self.sleep_count=0
	self.play_count=0
	self.counter=0
	self.stopFlag=1
	self.param=[]
	self.ball_detected = 'NULL'

#	self.param= rospy.get_param('/red_ball')
#	print(self.param[2])
#	self.param[2]='T'
#	rospy.set_param('/red_ball', self.param)
	#self.param[13]=T
#	rospy.set_param('home', self.param)
#	self.param= rospy.get_param('~home')
#	print(self.param[13])
        #wait some seconds when we launch the program
        time.sleep(6)
        smach.State.__init__(self, 
                             outcomes=['play','sleep'])


    def execute(self,userdata):

        rospy.loginfo('Executing state NORMAL ')
	#send to the actionlib client the target positions to reach
      	pub = rospy.Publisher('targetPosition', Num,queue_size=10)
	subscriber2=rospy.Subscriber("/cmd_vel", Twist, self.callback2)
        self.vel_pub = rospy.Publisher("cmd_vel",
                                       Twist, queue_size=1)
        # subscribed to the camera topic
        subscriber1= rospy.Subscriber("camera1/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size=1)


	self.var='FALSE'

	while(self.var=='FALSE'):
        	#send the robot random positions
		randomlist = []
		for i in range(0,2):
			n = random.randint(-6,8)
			randomlist.append(n)
	        rospy.loginfo('sending the random position: %s', randomlist)		
		pub.publish(randomlist)
		time.sleep(4)
		#se il robot si muove non mando i comandi
 		while(self.stopFlag==0):
			pass


		self.play_count = self.play_count+1
		#after some actions have been executed go to the sleep state
		if self.sleep_count==10 :
			self.sleep_count=0
#			self.var='FALSE'
			return user_action('SLEEP')
	
		if self.play_count==3 :
			self.play_count=0
#			self.var='FALSE'
			subscriber1.unregister()
			subscriber2.unregister()
			return user_action('PLAY')	


 
    def callback2(self, msg):
	#se il robot è fermo
#	print(msg.linear.x)
#	print(msg.angular.z)	
	if(msg.linear.x==0.0 and msg.angular.z==0.0):

		self.stopFlag=1
	
	else:
		self.stopFlag=0
		


    def callback(self,ros_data):
   
	#self.stopFlag=0
 #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

 #       greenLower = (50, 50, 50)#era 20 l uktimo
  #      greenUpper = (70, 255, 255)

        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
#if i detect the green
	greenLower = (50, 50, 50)#era 20 l uktimo
        greenUpper = (70, 255, 255)
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        #cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        # only proceed if at least one contour was found
	#put a flag to true when the robot sees the ball
        if len(cnts) > 0:
 #   		self.var = 'TRUE' 
		self.param = rospy.get_param('/LivingRoom')
		if (self.param[2] == 'T'):
			return
	        rospy.loginfo('green ball detected')
		self.ball_detected = self.param[3]
		#call the sub_track function
		self.sub_track(cnts, image_np, self.ball_detected)
		return	
#if i detect the black
	blackLower = (0, 0, 0)#era 20 l uktimo
        blackUpper = (5, 50, 50)
        mask = cv2.inRange(hsv, blackLower, blackUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        #cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        # only proceed if at least one contour was found
	#put a flag to true when the robot sees the ball
        if len(cnts) > 0:
 #   		self.var = 'TRUE' 
		self.param = rospy.get_param('/Bedroom')
		if (self.param[2] == 'T'):
			return
	        rospy.loginfo('black ball detected')
		self.ball_detected = self.param[3]
		self.sub_track(cnts, image_np, self.ball_detected)
		return	
#if i detect the red
	redLower = (0, 50, 50)#era 20 l uktimo
        redUpper = (5, 255, 255)
        mask = cv2.inRange(hsv, redLower, redUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        #cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        # only proceed if at least one contour was found
	#put a flag to true when the robot sees the ball
        if len(cnts) > 0:
 #   		self.var = 'TRUE' 
		self.param = rospy.get_param('/Closet')
		if (self.param[2] == 'T'):
			return
	        rospy.loginfo('red ball detected')
		self.ball_detected = self.param[3]
		self.sub_track(cnts, image_np, self.ball_detected)
		return	
#if i detect the yellow
	yellowLower = (25, 50, 50)#era 20 l uktimo
        yellowUpper = (35, 255, 255)
        mask = cv2.inRange(hsv, yellowLower, yellowUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        #cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        # only proceed if at least one contour was found
	#put a flag to true when the robot sees the ball
        if len(cnts) > 0:
 #   		self.var = 'TRUE' 
		self.param = rospy.get_param('/Kitchen')
		if (self.param[2] == 'T'):
			return
	        rospy.loginfo('yellow ball detected')
		self.ball_detected = self.param[3]
		self.sub_track(cnts, image_np, self.ball_detected)
		return	
#if i detect the blue
	blueLower = (100, 50, 50)#era 20 l uktimo
        blueUpper = (130, 255, 255)
        mask = cv2.inRange(hsv, blueLower, blueUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        #cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        # only proceed if at least one contour was found
	#put a flag to true when the robot sees the ball
        if len(cnts) > 0:
 #   		self.var = 'TRUE' 
		self.param = rospy.get_param('/Entrance')
		if (self.param[2] == 'T'):
			return
	        rospy.loginfo('blue ball detected')
		self.ball_detected = self.param[3]
		self.sub_track(cnts, image_np, self.ball_detected)
		return	
#if i detect the magenta
	magentaLower = (125, 50, 50)#era 20 l uktimo
        magentaUpper = (150, 255, 255)
        mask = cv2.inRange(hsv, magentaLower, magentaUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        #cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        # only proceed if at least one contour was found
	#put a flag to true when the robot sees the ball
        if len(cnts) > 0:
 #   		self.var = 'TRUE' 
		self.param = rospy.get_param('/Bathroom')
		if (self.param[2] == 'T'):
			return
	        rospy.loginfo('magenta ball detected')
		self.ball_detected = self.param[3]
		self.sub_track(cnts, image_np, self.ball_detected)
		return	


    def sub_track(self,cnts, image_np, ball_detected):

	    rospy.loginfo('sub_track function!')
	    self.counter=self.counter+1
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(image_np, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                vel = Twist()
                vel.angular.z = -0.002*(center[0]-400)
                vel.linear.x = -0.01*(radius-100)
                self.vel_pub.publish(vel)
		#if the robot is almost not moving register on the parameter server that the corresponding ball has been reached
		if (vel.angular.z<0.1 and vel.angular.z>-0.1 and vel.linear.x<0.1 and vel.linear.x>-0.1 or self.counter>200 ):
			rospy.loginfo('sono dalla pallinaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa')
			self.counter=0
			vel.angular.z=0.0
			vel.linear.x=0.0
                	self.vel_pub.publish(vel)
			if(ball_detected=='blue_ball'):
				self.param= rospy.get_param('/Entrance')
				self.param[2] = 'T'
				rospy.set_param('/Entrance', self.param)
				return
			if(ball_detected=='yellow_ball'):
				self.param= rospy.get_param('/Kitchen')
				self.param[2] = 'T'
				rospy.set_param('/Kitchen', self.param)
				return
			if(ball_detected=='magenta_ball'):
				self.param= rospy.get_param('/Bathroom')
				self.param[2] = 'T'
				rospy.set_param('/Bathroom', self.param)
				return
			if(ball_detected=='black_ball'):
				self.param= rospy.get_param('/Bedroom')
				self.param[2] = 'T'
				rospy.set_param('/Bedroom', self.param)
				return
			if(ball_detected=='green_ball'):
				self.param= rospy.get_param('/LivingRoom')
				self.param[2] = 'T'
				rospy.set_param('/LivingRoom', self.param)
				return
			if(ball_detected=='red_ball'):
				self.param= rospy.get_param('/Closet')
				self.param[2] = 'T'
				rospy.set_param('/Closet', self.param)
				return
				

            else:
                vel = Twist()
                vel.linear.x = 0.3
                self.vel_pub.publish(vel)

	    time.sleep(2/1000000.0)


				
        
   



#In the sleep state the robot goes to a predifined position and stays there for a certain time. After that it goes to the normal behavior

class Sleep(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['normal'])


        self.home = [-1,1]
	self.stopFlag=0

    def execute(self,userdata):
        rospy.loginfo('Executing state SLEEP')
	#send the actionlib client the target position to reach
        pub = rospy.Publisher('targetPosition', Num,queue_size=10) 
	rospy.Subscriber("/cmd_vel", Twist, self.callback)
	rospy.loginfo('sending the home position: %s', self.home)		
	pub.publish(self.home)	
        #rospy.wait_for_message('chatter', Int8)
	while(self.stopFlag==0):
		pass	
	#add a sleep to make the robot remain in the sleep state for a certain time
	time.sleep(20)
        
        return user_action('NORMAL')

    def callback(self, msg):
	#se il robot è fermo
#	rospy.loginfo("x,y,z")
#	print(msg.linear.x)
#	print(msg.angular.z)	
	if(msg.linear.x<0.03 and msg.linear.x>-0.03 and msg.angular.z<0.03 and msg.angular.z>-0.03):

		self.stopFlag=1
	
	else:
		self.stopFlag=0
               
            




class Play(smach.State):

    def __init__(self):
	
        smach.State.__init__(self, 
                             outcomes=['normal', 'find'],
                             output_keys=['room_out'])
        self.var2=0
	self.count=0
	self.play_pose=[-5, 8]
	self.rooms=[]
	self.param=[]
	self.stopFlag=0
                          

    def execute(self,userdata):
	rospy.loginfo('Executing state PLAY')
	pub = rospy.Publisher('targetPosition', Num,queue_size=10) 
	subscriber=rospy.Subscriber("/cmd_vel", Twist, self.callback)


	while(self.count<2):
		self.count=self.count+1
		playPosition= rospy.get_param('/playPose')
		lista=[]
		lista.append(playPosition[0])
		lista.append(playPosition[1])
		pub.publish(lista)
		rospy.loginfo('going to the play pose')		
		time.sleep(4)
		#waint until the robot stop moving 
		while(self.stopFlag==0):
			pass
		self.rooms= rospy.get_param('/rooms')
		n = random.randint(0,5)
		print(n)
		self.goTo= self.rooms[n]
		rospy.loginfo('goTo command: %s', self.goTo)
		if(self.goTo=='Closet'):
			self.param= rospy.get_param('/Closet')
			if(self.param[2]=='T'):
				lista=[]
				lista.append(self.param[0])
				lista.append(self.param[1])
				rospy.loginfo('sending the room position: %s', lista)		
				pub.publish(lista)
			#move to the find state
			else: 	
				self.count=0
				subscriber.unregister()
				userdata.room_out=self.param[3]
				return user_action('FIND')	

		elif(self.goTo=='LivingRoom'):
			self.param= rospy.get_param('/LivingRoom')
			if(self.param[2]=='T'):
				lista=[]
				lista.append(self.param[0])
				lista.append(self.param[1])
				rospy.loginfo('sending the room position: %s', lista)		
				pub.publish(lista)
			else: 	
				self.count=0
				subscriber.unregister()
				userdata.room_out=self.param[3]
				return user_action('FIND')

		elif(self.goTo=='Entrance'):
			self.param= rospy.get_param('/Entrance')
			if(self.param[2]=='T'):
				lista=[]
				lista.append(self.param[0])
				lista.append(self.param[1])
				rospy.loginfo('sending the room position: %s', lista)		
				pub.publish(lista)
			else: 
				self.count=0
				subscriber.unregister()	
				userdata.room_out=self.param[3]
				return user_action('FIND')

		elif(self.goTo=='Bedroom'):
			self.param= rospy.get_param('/Bedroom')
			if(self.param[2]=='T'):
				lista=[]
				lista.append(self.param[0])
				lista.append(self.param[1])
				rospy.loginfo('sending the room position: %s', lista)		
				pub.publish(lista)
			else: 	
				self.count=0
				subscriber.unregister()
				userdata.room_out=self.param[3]
				return user_action('FIND')

		elif(self.goTo=='Bathroom'):
			self.param= rospy.get_param('/Bathroom')
			if(self.param[2]=='T'):
				lista=[]
				lista.append(self.param[0])
				lista.append(self.param[1])
				rospy.loginfo('sending the room position: %s', lista)		
				pub.publish(lista)
			else:
				self.count=0 
				subscriber.unregister()	
				userdata.room_out= self.param[3]
				return user_action('FIND')

		elif(self.goTo=='Kitchen'):
			self.param= rospy.get_param('/Kitchen')
			if(self.param[2]=='T'):
				lista=[]
				lista.append(self.param[0])
				lista.append(self.param[1])
				rospy.loginfo('sending the room: %s', lista)		
				pub.publish(lista)
			else: 	
				self.count=0
				subscriber.unregister()
				userdata.room_out= self.param[3]
				return user_action('FIND')

		time.sleep(2)
		#waint until the robot stop moving 
		while(self.stopFlag==0):
			pass
		time.sleep(5)
	subscriber.unregister()
	self.count=0
	return user_action('NORMAL')	


    def callback(self, msg):
	#se il robot è fermo
#	rospy.loginfo("x,y,z")
#	print(msg.linear.x)
#	print(msg.angular.z)	
	if(msg.linear.x==0 and msg.angular.z==0):

		self.stopFlag=1
	
	else:
		self.stopFlag=0



class Find(smach.State):

    def __init__(self):
	
        smach.State.__init__(self, 
                             outcomes=['play'],
                             input_keys=['room_in'])
        self.var2=0
	self.counter=0
	self.play_pose=[-5, 8]
	self.rooms=[]
	self.param=[]
	self.stopFlag=0

	self.play_state=False

    def execute(self,userdata):

	rospy.loginfo('Executing state FIND (room received=%s)' %userdata.room_in)
	rospy.Subscriber("/cmd_vel", Twist, self.callback)
	pub = rospy.Publisher('targetPosition', Num,queue_size=10)
	self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # subscribed to the camera topic
        subscriber1= rospy.Subscriber("camera1/image_raw/compressed",
                                           CompressedImage, self.callback2,  queue_size=1)
	n=0 
	self.requested_room=userdata.room_in

	#choose randomly a room and check that it has not been discovered yet. If not go there, then check if it corresponds to the user target
#	self.rooms= rospy.get_param('/rooms')
#	n = random.randint(0,5)
#	print(n)
#	self.goTo= self.rooms[n]



	while(self.play_state==False):
        	#send the robot random positions
		randomlist = []
		for i in range(0,2):
			n = random.randint(-6,8)
			randomlist.append(n)
	        rospy.loginfo('sending the random position: %s', randomlist)		
		pub.publish(randomlist)
		time.sleep(4)
		#se il robot si muove non mando i comandi
 		while(self.stopFlag==0):
			pass

	self.play_state=False
	return user_action('PLAY')
	


 		




    def callback2(self,ros_data):

	#self.stopFlag=0
 #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

 #       greenLower = (50, 50, 50)#era 20 l uktimo
  #      greenUpper = (70, 255, 255)

        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
#if i detect the green
	greenLower = (50, 50, 50)#era 20 l uktimo
        greenUpper = (70, 255, 255)
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        #cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        # only proceed if at least one contour was found
	#put a flag to true when the robot sees the ball
        if len(cnts) > 0:
#   		self.var = 'TRUE' 
		self.param = rospy.get_param('/LivingRoom')
		if (self.param[2] == 'T'):
			return
 	        rospy.loginfo('green ball detected')
		self.ball_detected = self.param[3]
		#call the sub_track function
		self.sub_track(cnts, image_np, self.ball_detected)
		return	
#if i detect the black
	blackLower = (0, 0, 0)#era 20 l uktimo
        blackUpper = (5, 50, 50)
        mask = cv2.inRange(hsv, blackLower, blackUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        #cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        # only proceed if at least one contour was found
	#put a flag to true when the robot sees the ball
        if len(cnts) > 0:
#   		self.var = 'TRUE' 
		self.param = rospy.get_param('/Bedroom')
		if (self.param[2] == 'T'):
			return

	        rospy.loginfo('black ball detected')
		self.ball_detected = self.param[3]
		self.sub_track(cnts, image_np, self.ball_detected)
		return	
#if i detect the red
	redLower = (0, 50, 50)#era 20 l uktimo
        redUpper = (5, 255, 255)
        mask = cv2.inRange(hsv, redLower, redUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        #cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        # only proceed if at least one contour was found
	#put a flag to true when the robot sees the ball
        if len(cnts) > 0:
#   		self.var = 'TRUE' 
		self.param = rospy.get_param('/Closet')
		if (self.param[2] == 'T'):
			return
	        rospy.loginfo('red ball detected')
		self.ball_detected = self.param[3]
		self.sub_track(cnts, image_np, self.ball_detected)
		return	
#if i detect the yellow
	yellowLower = (25, 50, 50)#era 20 l uktimo
        yellowUpper = (35, 255, 255)
        mask = cv2.inRange(hsv, yellowLower, yellowUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        #cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        # only proceed if at least one contour was found
	#put a flag to true when the robot sees the ball
        if len(cnts) > 0:
#   		self.var = 'TRUE' 
		self.param = rospy.get_param('/Kitchen')
		if (self.param[2] == 'T'):
			return

	        rospy.loginfo('yellow ball detected')
		self.ball_detected = self.param[3]
		self.sub_track(cnts, image_np, self.ball_detected)
		return	
#if i detect the blue
	blueLower = (100, 50, 50)#era 20 l uktimo
        blueUpper = (130, 255, 255)
        mask = cv2.inRange(hsv, blueLower, blueUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        #cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        # only proceed if at least one contour was found
	#put a flag to true when the robot sees the ball
        if len(cnts) > 0:
#   		self.var = 'TRUE' 
		self.param = rospy.get_param('/Entrance')
		if (self.param[2] == 'T'):
			return

	        rospy.loginfo('blue ball detected')
		self.ball_detected = self.param[3]
		self.sub_track(cnts, image_np, self.ball_detected)
		return	
#if i detect the magenta
	magentaLower = (125, 50, 50)#era 20 l uktimo
        magentaUpper = (150, 255, 255)
        mask = cv2.inRange(hsv, magentaLower, magentaUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        #cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        # only proceed if at least one contour was found
	#put a flag to true when the robot sees the ball
        if len(cnts) > 0:
#   		self.var = 'TRUE' 
		self.param = rospy.get_param('/Bathroom')
		if (self.param[2] == 'T'):
			return

	        rospy.loginfo('magenta ball detected')
		self.ball_detected = self.param[3]
		self.sub_track(cnts, image_np, self.ball_detected)
		return	


    def sub_track(self,cnts, image_np, ball_detected):

	    rospy.loginfo('sub_track function!')
	    self.counter=self.counter+1
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(image_np, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                vel = Twist()
                vel.angular.z = -0.002*(center[0]-400)
                vel.linear.x = -0.01*(radius-100)
                self.vel_pub.publish(vel)
		#if the robot is almost not moving register on the parameter server that the corresponding ball has been reached
		if (vel.angular.z<0.1 and vel.angular.z>-0.1 and vel.linear.x<0.1 and vel.linear.x>-0.1 or self.counter>100 ):
			rospy.loginfo('sono dalla pallinaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa')
			self.counter=0

			if(ball_detected=='blue_ball'):
				rospy.loginfo('1')
				self.param= rospy.get_param('/Entrance')
				self.param[2] = 'T'
				rospy.set_param('/Entrance', self.param)
				if(self.requested_room==ball_detected):
					rospy.loginfo('SONO DENTRO')
					self.play_state=True
					time.sleep(1)
				return
			if(ball_detected=='yellow_ball'):
				rospy.loginfo('1')
				self.param= rospy.get_param('/Kitchen')
				self.param[2] = 'T'
				rospy.set_param('/Kitchen', self.param)
				if(self.requested_room==ball_detected):
					rospy.loginfo('SONO DENTRO')
					self.play_state=True
			
					time.sleep(1)
				return
			if(ball_detected=='magenta_ball'):
				rospy.loginfo('1')
				self.param= rospy.get_param('/Bathroom')
				self.param[2] = 'T'
				rospy.set_param('/Bathroom', self.param)
				if(self.requested_room==ball_detected):
					rospy.loginfo('SONO DENTRO')
					self.play_state=True
			
					time.sleep(1)
				return
			if(ball_detected=='black_ball'):
				rospy.loginfo('1')
				self.param= rospy.get_param('/Bedroom')
				self.param[2] = 'T'
				rospy.set_param('/Bedroom', self.param)
				if(self.requested_room==ball_detected):
					rospy.loginfo('SONO DENTRO')
					self.play_state=True

					time.sleep(1)
				return
			if(ball_detected=='green_ball'):
				rospy.loginfo('1')
				self.param= rospy.get_param('/LivingRoom')
				self.param[2] = 'T'
				rospy.set_param('/LivingRoom', self.param)
				if(self.requested_room==ball_detected):
					rospy.loginfo('SONO DENTRO')
					self.play_state=True
			
					time.sleep(1)
				return
			if(ball_detected=='red_ball'):
				rospy.loginfo('1')
				self.param= rospy.get_param('/Closet')
				self.param[2] = 'T'
				rospy.set_param('/Closet', self.param)
				if(self.requested_room==ball_detected):
					rospy.loginfo('SONO DENTRO')
					self.play_state=True
			
					time.sleep(1)
				return
				

            else:
                vel = Twist()
                vel.linear.x = 0.3
                self.vel_pub.publish(vel)

	    time.sleep(2/1000000.0)

	


    def callback(self, msg):
	#se il robot è fermo
#	rospy.loginfo("x,y,z")
#	print(msg.linear.x)
#	print(msg.angular.z)	
	if(msg.linear.x==0 and msg.angular.z==0):

		self.stopFlag=1
	
	else:
		self.stopFlag=0
					
			
	
	

  





class func():
  def __init__(self):
        
                       
    rospy.init_node('state_machine') 

  
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
   

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={
                                            'play':'PLAY',
				             'sleep' : 'SLEEP'})
                                              
        smach.StateMachine.add('PLAY', Play(), 
                               transitions={'normal':'NORMAL',
					    'find':'FIND'},
			       remapping={'room_out':'sm_room'}) 
                                            
	smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'normal':'NORMAL'})

	smach.StateMachine.add('FIND', Find(), 
				transitions={'play':'PLAY'},
			       remapping={'room_in':'sm_room'}) 
						
                               
                               

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

 

if __name__ == '__main__':
    
    func()
