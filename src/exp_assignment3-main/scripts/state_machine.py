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
import subprocess
import signal

from actionlib import GoalID
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

   

##In the normal state the robot reaches random positions. When it detects a new colored object
#it enters the track function. After some times move to the Sleep or Play behaviour
class Normal(smach.State):



    def __init__(self):

	self.sleep_count=0
	self.play_count=0
	self.counter=0
	self.stopFlag=1
	self.param=[]
	self.ball_detected = 'NULL'

        time.sleep(6)
        smach.State.__init__(self, 
                             outcomes=['play','sleep'])


    def execute(self,userdata):

        rospy.loginfo('Executing state NORMAL ')

      	pub = rospy.Publisher('targetPosition', Num,queue_size=10)
	subscriber2=rospy.Subscriber("cmd_vel", Twist, self.callback2)
        self.vel_pub = rospy.Publisher("cmd_vel",
                                       Twist, queue_size=1)
        # subscribe to the camera topic
        subscriber1= rospy.Subscriber("camera1/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size=1)


	self.var='FALSE'

	while(self.var=='FALSE'):
        	#send the robot random positions
		randomlist = []
		for i in range(0,2):
			n = random.randint(-6,8)
			randomlist.append(n)
	        rospy.loginfo('moving to the random position: %s', randomlist)		
		pub.publish(randomlist)
		time.sleep(4)

		#stop sending commands when the robot is moving
 		while(self.stopFlag==0):
			pass

		self.play_count = self.play_count+1
		self.sleep_count = self.sleep_count+1
		#after some actions have been executed go to the sleep state
		if self.sleep_count>=6 :
			self.sleep_count=0
			subscriber1.unregister()
			subscriber2.unregister()
			return user_action('SLEEP')
		#after some actions have been executed go to the play state
		if self.play_count>=3 :
			self.play_count=0
			subscriber1.unregister()
			subscriber2.unregister()
			return user_action('PLAY')	


    ## A flag is set to 1 if the robot is not moving, to 0 otherwise	 
    def callback2(self, msg):
	if(msg.linear.x==0.0 and msg.angular.z==0.0):

		self.stopFlag=1
	
	else:
		self.stopFlag=0
		

    ##The program enters here everytime e new image is available from the camera. An algorithm is implemented
    #to detect six different colored balls.
    def callback(self,ros_data):

   
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) 

        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
	#if i detect the green
	greenLower = (50, 50, 50)
        greenUpper = (70, 255, 255)
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        # only proceed if at least one contour was found
        if len(cnts) > 0:
		self.param = rospy.get_param('/LivingRoom')
		if (self.param[2] == 'T'):
			return
	        rospy.loginfo('green ball detected')
		self.ball_detected = self.param[3]
		#call the sub_track function
		self.sub_track(cnts, image_np, self.ball_detected)
		return	
	#if i detect the black
	blackLower = (0, 0, 0)
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
        if len(cnts) > 0:
		self.param = rospy.get_param('/Bedroom')
		if (self.param[2] == 'T'):
			return
	        rospy.loginfo('black ball detected')
		self.ball_detected = self.param[3]
		self.sub_track(cnts, image_np, self.ball_detected)
		return	
	#if i detect the red
	redLower = (0, 50, 50)
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
        if len(cnts) > 0:
		self.param = rospy.get_param('/Closet')
		if (self.param[2] == 'T'):
			return
	        rospy.loginfo('red ball detected')
		self.ball_detected = self.param[3]
		self.sub_track(cnts, image_np, self.ball_detected)
		return	
	#if i detect the yellow
	yellowLower = (25, 50, 50)
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
        if len(cnts) > 0:
		self.param = rospy.get_param('/Kitchen')
		if (self.param[2] == 'T'):
			return
	        rospy.loginfo('yellow ball detected')
		self.ball_detected = self.param[3]
		self.sub_track(cnts, image_np, self.ball_detected)
		return	
	#if i detect the blue
	blueLower = (100, 50, 50)
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
        if len(cnts) > 0:
		self.param = rospy.get_param('/Entrance')
		if (self.param[2] == 'T'):
			return
	        rospy.loginfo('blue ball detected')
		self.ball_detected = self.param[3]
		self.sub_track(cnts, image_np, self.ball_detected)
		return	
	#if i detect the magenta
	magentaLower = (125, 50, 50)
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
        if len(cnts) > 0:
		self.param = rospy.get_param('/Bathroom')
		if (self.param[2] == 'T'):
			return
	        rospy.loginfo('magenta ball detected')
		self.ball_detected = self.param[3]
		self.sub_track(cnts, image_np, self.ball_detected)
		return	

    ##This function is executed everytime the robot detects a color object which was not previously detected.
    #The robot tries to go closer to the detected object. Once done, the reference position of the room is
    #marked as reached modifying data in the parameter server.
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
		if ( vel.linear.x<0.1 and vel.linear.x>-0.1 or self.counter>250 ):
			rospy.loginfo('New room position stored!!')
			self.counter=0
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



				 


##The robot reaches a predefined location obtained accessing the parameter server. 
#Once reached it stays there for some time and then move to the normal behaviour.
class Sleep(smach.State):


    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['normal'])


	self.stopFlag=0

    def execute(self,userdata):
        rospy.loginfo('Executing state SLEEP')
	#send the actionlib client the target position to reach
        pub = rospy.Publisher('targetPosition', Num,queue_size=10) 
	rospy.Subscriber("cmd_vel", Twist, self.callback)
	home= rospy.get_param('/homePose')
	lista=[]
	lista.append(home[0])
	lista.append(home[1])
	pub.publish(lista)
	rospy.loginfo('Going to the home position')		
	time.sleep(3)
	while(self.stopFlag==0):
		pass	
	#add a sleep to make the robot remain in the sleep state for a certain time
	time.sleep(20)
        
        return user_action('NORMAL')

    ## A flag is set to 1 if the robot is not moving, to 0 otherwise	
    def callback(self, msg):
	
	if(msg.linear.x<0.03 and msg.linear.x>-0.03 and msg.angular.z<0.03 and msg.angular.z>-0.03):

		self.stopFlag=1
	
	else:
		self.stopFlag=0
               
            



##The robot goes to a predefined location and waits to receive a goTo command 
#which is extracted randomly from the parameter server.
#If the room contained in the command has already been discovered by the robot
#it moves there. Otherwise it switches to the find state sending as parameter the color
#of the ball present in the just mentioned room.
class Play(smach.State):


    def __init__(self):
	
        smach.State.__init__(self, 
                             outcomes=['normal', 'find'],
                             output_keys=['room_out'])

	self.count=0
	self.rooms=[]
	self.param=[]
	self.stopFlag=0
                          

    def execute(self,userdata):
	rospy.loginfo('Executing state PLAY')
	pub = rospy.Publisher('targetPosition', Num,queue_size=10) 
	subscriber=rospy.Subscriber("cmd_vel", Twist, self.callback)


	while(self.count<2):
		self.count=self.count+1
		playPosition= rospy.get_param('/playPose')
		lista=[]
		lista.append(playPosition[0])
		lista.append(playPosition[1])
		pub.publish(lista)
		rospy.loginfo('going to the play pose')		
		time.sleep(4)
		#wait until the robot stop moving 
		while(self.stopFlag==0):
			pass
		self.rooms= rospy.get_param('/rooms')
		n = random.randint(0,5)
		self.goTo= self.rooms[n]
		rospy.loginfo('goTo command: %s', self.goTo)
		if(self.goTo=='Closet'):
			self.param= rospy.get_param('/Closet')
			if(self.param[2]=='T'):
				lista=[]
				lista.append(self.param[0])
				lista.append(self.param[1])
				rospy.loginfo('going to the predefined room position: %s', lista)		
				pub.publish(lista)
			#move to the find state
			else: 	
				subscriber.unregister()
				userdata.room_out=self.param[3]
				return user_action('FIND')	

		elif(self.goTo=='LivingRoom'):
			self.param= rospy.get_param('/LivingRoom')
			if(self.param[2]=='T'):
				lista=[]
				lista.append(self.param[0])
				lista.append(self.param[1])
				rospy.loginfo('going to the predefined room position: %s', lista)		
				pub.publish(lista)
			else: 	
				subscriber.unregister()
				userdata.room_out=self.param[3]
				return user_action('FIND')

		elif(self.goTo=='Entrance'):
			self.param= rospy.get_param('/Entrance')
			if(self.param[2]=='T'):
				lista=[]
				lista.append(self.param[0])
				lista.append(self.param[1])
				rospy.loginfo('going to the predefined room position: %s', lista)		
				pub.publish(lista)
			else: 
				subscriber.unregister()	
				userdata.room_out=self.param[3]
				return user_action('FIND')

		elif(self.goTo=='Bedroom'):
			self.param= rospy.get_param('/Bedroom')
			if(self.param[2]=='T'):
				lista=[]
				lista.append(self.param[0])
				lista.append(self.param[1])
				rospy.loginfo('going to the predefined room position: %s', lista)		
				pub.publish(lista)
			else: 	
				subscriber.unregister()
				userdata.room_out=self.param[3]
				return user_action('FIND')

		elif(self.goTo=='Bathroom'):
			self.param= rospy.get_param('/Bathroom')
			if(self.param[2]=='T'):
				lista=[]
				lista.append(self.param[0])
				lista.append(self.param[1])
				rospy.loginfo('going to the predefined room position: %s', lista)		
				pub.publish(lista)
			else:
				subscriber.unregister()	
				userdata.room_out= self.param[3]
				return user_action('FIND')

		elif(self.goTo=='Kitchen'):
			self.param= rospy.get_param('/Kitchen')
			if(self.param[2]=='T'):
				lista=[]
				lista.append(self.param[0])
				lista.append(self.param[1])
				rospy.loginfo('going to the predefined room position: %s', lista)		
				pub.publish(lista)
			else: 	
				subscriber.unregister()
				userdata.room_out= self.param[3]
				return user_action('FIND')

		time.sleep(2)
		#wait until the robot stop moving 
		while(self.stopFlag==0):
			pass
		time.sleep(5)
	subscriber.unregister()
	self.count=0
	return user_action('NORMAL')	

    ##A flag is set to 1 if the robot is not moving, to 0 otherwise.	
    def callback(self, msg):

	if(msg.linear.x==0 and msg.angular.z==0):

		self.stopFlag=1
	
	else:
		self.stopFlag=0


##The robot starts moving in the environment relying on the explore-lite package. Once it detects a new ball
#it checks if it corresponds to the ball received by the play state. If so it goes back
#to the play behaviour. Otherwise it keeps moving around the house.
class Find(smach.State):


    def __init__(self):
	
        smach.State.__init__(self, 
                             outcomes=['play'],
                             input_keys=['room_in'])

	self.counter=0
	self.param=[]
	self.stopFlag=0
	self.play_state=False
	self.child=None

    def execute(self,userdata):

	rospy.loginfo('Executing state FIND (room received=%s)' %userdata.room_in)

	pub = rospy.Publisher('targetPosition', Num,queue_size=10)
	self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # subscribed to the camera topic
        subscriber= rospy.Subscriber("camera1/image_raw/compressed",
                                          CompressedImage, self.callback2,  queue_size=1)

	self.requested_room=userdata.room_in
	#launch the explore-lite package everytime the robot enters the play state
	self.child = subprocess.Popen(["roslaunch","explore_lite","explore.launch"])
	self.t_final=time.time() + 200



	while(self.play_state==False and time.time()<self.t_final):

      		pass




	time.sleep(2)
	self.child.send_signal(signal.SIGINT)
	subscriber.unregister()
	self.play_state=False
	return user_action('PLAY')
	


 		



    ##The program enters here everytime e new image is available from the camera. An algorithm is implemented
    #to detect six different colored ball.
    def callback2(self,ros_data):


        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  

        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
	#if i detect the green
	greenLower = (50, 50, 50)
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
        if len(cnts) > 0:
		self.param = rospy.get_param('/LivingRoom')
		if (self.param[2] == 'T'):
			return
 	        rospy.loginfo('green ball detected')
		self.t_final=time.time() + 50
		self.ball_detected = self.param[3]
		self.sub_track(cnts, image_np, self.ball_detected)
		return	
	#if i detect the black
	blackLower = (0, 0, 0)
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
        if len(cnts) > 0:
		self.param = rospy.get_param('/Bedroom')
		if (self.param[2] == 'T'):
			return

	        rospy.loginfo('black ball detected')
		self.ball_detected = self.param[3]
		self.t_final=time.time() + 50
		self.sub_track(cnts, image_np, self.ball_detected)
		return	
	#if i detect the red
	redLower = (0, 50, 50)
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
        if len(cnts) > 0:
		self.param = rospy.get_param('/Closet')
		if (self.param[2] == 'T'):
			return
	        rospy.loginfo('red ball detected')
		self.ball_detected = self.param[3]
		self.t_final=time.time() + 150
		self.sub_track(cnts, image_np, self.ball_detected)
		return	
	#if i detect the yellow
	yellowLower = (25, 50, 50)
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
        if len(cnts) > 0:
		self.param = rospy.get_param('/Kitchen')
		if (self.param[2] == 'T'):
			return

	        rospy.loginfo('yellow ball detected')
		self.t_final=time.time() + 50
		self.ball_detected = self.param[3]
		self.sub_track(cnts, image_np, self.ball_detected)
		return	
	#if i detect the blue
	blueLower = (100, 50, 50)
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
        if len(cnts) > 0:
		self.param = rospy.get_param('/Entrance')
		if (self.param[2] == 'T'):
			return

	        rospy.loginfo('blue ball detected')
		self.t_final=time.time() + 50
		self.ball_detected = self.param[3]
		self.sub_track(cnts, image_np, self.ball_detected)
		return	
	#if i detect the magenta
	magentaLower = (125, 50, 50)
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
        if len(cnts) > 0:
		self.param = rospy.get_param('/Bathroom')
		if (self.param[2] == 'T'):
			return

	        rospy.loginfo('magenta ball detected')
		self.t_final=time.time() + 50
		self.ball_detected = self.param[3]
		self.sub_track(cnts, image_np, self.ball_detected)
		return	

    ##This function is executed everytime the robot detects a colored object which was not previously detected.
    #The robot tries to go closer to the detected object. Once done, the reference position of the room is
    #marked as reached modifying data in the parameter server. Then the robot checks if the detected ball corresponds
    #to the ball received from the play state.
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
		if ( vel.linear.x<0.1 and vel.linear.x>-0.1 or self.counter>250 ):
			rospy.loginfo('New room position stored!!')
			self.counter=0

			if(ball_detected=='blue_ball'):
				self.param= rospy.get_param('/Entrance')
				self.param[2] = 'T'
				rospy.set_param('/Entrance', self.param)
				if(self.requested_room==ball_detected):
					self.play_state=True
					time.sleep(1)


				return

			if(ball_detected=='yellow_ball'):
				self.param= rospy.get_param('/Kitchen')
				self.param[2] = 'T'
				rospy.set_param('/Kitchen', self.param)
				if(self.requested_room==ball_detected):
					self.play_state=True
			
					time.sleep(1)

				return

			if(ball_detected=='magenta_ball'):
				self.param= rospy.get_param('/Bathroom')
				self.param[2] = 'T'
				rospy.set_param('/Bathroom', self.param)
				if(self.requested_room==ball_detected):
					self.play_state=True
			
					time.sleep(1)

				return

			if(ball_detected=='black_ball'):
				self.param= rospy.get_param('/Bedroom')
				self.param[2] = 'T'
				rospy.set_param('/Bedroom', self.param)
				if(self.requested_room==ball_detected):
					self.play_state=True

					time.sleep(1)
				return

			if(ball_detected=='green_ball'):
				self.param= rospy.get_param('/LivingRoom')
				self.param[2] = 'T'
				rospy.set_param('/LivingRoom', self.param)
				if(self.requested_room==ball_detected):
					self.play_state=True
			
					time.sleep(1)

				return

			if(ball_detected=='red_ball'):
				self.param= rospy.get_param('/Closet')
				self.param[2] = 'T'
				rospy.set_param('/Closet', self.param)
				if(self.requested_room==ball_detected):
					self.play_state=True
			
					time.sleep(1)

				return
				

            else:
                vel = Twist()
                vel.linear.x = 0.3
                self.vel_pub.publish(vel)


	
					
			


##A state machine with 4 states is initialized. 
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
