#!/usr/bin/env python3


#Importing required Packages
import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu,LaserScan,Range
from geometry_msgs.msg import Twist, Point, Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64
import math
import numpy as np 
import time

#Class defined encapsulationg variables and functions nessecary for Traversal
class Traverse:

	#The init function executed every time the constructor/Class name is called in main:

	def __init__(self,x,y):
		self.goal_x = x
		self.goal_y = y
		self.obs = []
		self.speed = Twist()
		self.sub_gps = rospy.Subscriber('/odom', Odometry, self.Odom) #Subscriber to the topic /fix
		print("Goal Recieved : "+self.goal_x+" , " + self.goal_y)
		R = rospy.Rate(1)
		R.sleep()
		self.sub2 = rospy.Subscriber("/ebot/laser/scan",LaserScan,self.get_laser)
		self.sub = rospy.Subscriber('/imu', Imu, self.Angle)	     #Subscriber to the /imu topic
		self.pub = rospy.Publisher('/cmd_vel',Twist, queue_size = 1) #Publisher to /cmd_vel topic
		


	#Function to correct the angle and convert range from (-180,180) to (0,360):	
	def Angle_correction(self,angle):
		if(angle<0):
			angle_conv = 360+angle
		else:
			angle_conv = angle

		return angle_conv

	#Function to find the shortest angle the rover has to rotate with the angle difference as the input parameter

	def Short_Angle(self,theta_i):
		if(theta_i<0):
			if(math.fabs(theta_i)>180):	#Case 1 : Large and Negative
				theta_corr = 360+theta_i
				self.fac = 1
			else:				#Case 2 : Small and negative
				theta_corr = theta_i
				self.fac = -1
		else:
			if(theta_i>180):		#Case 3 : Large and Positive
				theta_corr = 360-theta_i
				self.fac =-1
			else:				#Case 4 : Small and Positive
				theta_corr = theta_i
				self.fac =1
		return theta_corr

	#Callback Function of GPS Subscriber to obtain the latitude,longitude and altitude from the /fix topic's message and use Pyproj to calculate fwd azimuthal angle and distance:

	def Odom(self,req):
		self.x_co = req.pose.pose.position.x
		self.y_co = req.pose.pose.position.y
		self.z_co = req.pose.pose.position.z
		self.diff_x = self.goal_x-self.x_co
		self.diff_y = self.goal_y-self.y_co
		self.dist = math.sqrt((self.diff_x)**2+(self.diff_y)**2)
		rospy.loginfo("Distance from goal: %lf",self.dist)
		self.theta_input = (math.degrees(math.atan(self.diff_x/self.diff_y)))
	
	#Function that sets the linear velocity of the rover to 0.2 when it is aligned and at a distance greater than 0.2 m (Forward Traversal):

	def Straight(self):
		Distance = self.dist
		rospy.loginfo("Distance from goal: %lf",Distance)
		if(Distance>1):
			self.speed.linear.x=0.5
			self.pub.publish(self.speed)

		else:
			self.speed.linear.x=0
			self.pub.publish(self.speed)
			print("Traversal Complete")
			rospy.signal_shutdown("Traversal Complete") #Shuts the node down once Traversal is complete

	#Function to rotate the rover to the desired heading dynamically calculating angle difference and setting angular velocity (+/-) accordingly			

	def Turn(self):
		self.angle = math.degrees(self.yaw)
		#rospy.loginfo("Fwd Angle : %lf, yaw: %lf",self.fwd_angle,self.angle)
		rospy.loginfo("corrected angle diff: %lf",self.theta_input)
		self.theta_input = self.Angle_correction(self.theta_input-self.angle)
		self.theta = self.Short_Angle(self.theta_input)
		rospy.loginfo("corrected shortest angle diff: %lf",self.theta)
		if(math.fabs((self.theta))>1.25):
			self.speed.angular.z=0.5*(self.fac)
			self.pub.publish(self.speed)
		else:
			self.speed.angular.z=0
			self.pub.publish(self.speed)
			print("Turning Complete")
			self.Straight()
			#self.Flag = False

	#Callback function of the IMU Subscriber to Obtain the orientation of the rover in real time and execute turning or traversal:

	def Angle(self,msg):
		self.x = msg.orientation.x
		self.y = msg.orientation.y
		self.z = msg.orientation.z
		self.w = msg.orientation.w
		(self.roll, self.pitch, self.yaw) = euler_from_quaternion([self.x,self.y,self.z,self.w])
		if(self.dist>1):
			if(self.obs[1] or self.obs[0] or self.obs[2]):
				self.avoid()
			else:
				#self.speed.linear.x = 0
				#self.speed.angular.z = 0
				#self.pub.publish(self.speed)
				self.Turn()
		else:
			self.speed.angular.z=0
			self.pub.publish(self.speed)
			print("Turning Complete")
			self.Straight()
#		if(self.Flag):
#			self.Turn()
#		else:
#			print("Rover Aligned. Executing Traversal forward")
#			self.Straight()
	def avoid(self):
		if(self.obs[0] and not self.obs[1] and not self.obs[2]): # TFF
			self.speed.linear.x = 0.5
			self.speed.angular.z = 0.5
			self.pub.publish(self.speed)
		elif(self.obs[1] and not self.obs[0] and not self.obs[2]): # FTF
			self.speed.linear.x = 0.5
			self.speed.angular.z = 0.5
			self.pub.publish(self.speed)
		elif(self.obs[2] and not self.obs[1] and not self.obs[0]): # FFT
			self.speed.linear.x = 0.5
			self.speed.angular.z = -0.5
			self.pub.publish(self.speed)
		elif(self.obs[1] and self.obs[2] and not self.obs[0]): #FTT
			self.speed.linear.x = 0
			self.speed.angular.z = -0.5
			self.pub.publish(self.speed)
		elif(self.obs[0] and self.obs[2] and not self.obs[1]): #TFT
			self.speed.linear.x = 0.5
			self.speed.angular.z = 0
			self.pub.publish(self.speed)
		elif(self.obs[1] and self.obs[0] and not self.obs[2]): # TTF
			self.speed.linear.x = 0
			self.speed.angular.z = 0.5
			self.pub.publish(self.speed)
		elif(self.obs[1] and self.obs[0] and self.obs[2]):	#TTT
			self.speed.linear.x = 0
			self.speed.angular.z = 0.5
			self.pub.publish(self.speed)
		else:							#FFF
			self.speed.linear.x = 0.5
			self.speed.angular.z = 0
			self.pub.publish(self.speed)
	def get_laser(self,msg):
		tot = 720
		fac = 3
		Iter = tot/fac
		Range = 0
		max_range = msg.range_max
		#print(max_range)
		self.speed = Twist()
		values =[]
		for i in range(0,fac):
			values.append(min(min(msg.ranges[int(Range):int(Range+Iter)]),max_range))
			Range+=Iter
		#print(values)
		self.obs = {}
		for i in range(0,fac):
			if(values[i]<1.25):
				self.obs.update({i : True})
			else:
				self.obs.update({i : False})
		#print(self.obs)
#Test Cases
#Goal 1:
#x:  -1.3748298229342164
#y: 7.584105298605847

	
#Main function iterated repeatedly using rospy.spin() 

if __name__ == '__main__':
    rospy.init_node('IMU')
    x = np.float64(input("Enter goal x : "))	#Obtaining goal latitude from user
    y = np.float64(input("Enter goal y : ")) #Obtaining goal longitude from user
    Traverse(x,y) 				#Constructor of the class called
    rospy.spin()
