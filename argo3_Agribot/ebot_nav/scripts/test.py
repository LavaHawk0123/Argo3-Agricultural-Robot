#!/usr/bin/env python3


#Importing required Packages
import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu,LaserScan,Range
from geometry_msgs.msg import Twist, Point, Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64
import pyproj
import math
import numpy as np 
import time

#Class defined encapsulationg variables and functions nessecary for Traversal
class Traverse:

	#The init function executed every time the constructor/Class name is called in main:

	def __init__(self,x,y):
		self.goal_x = x
		self.goal_y = y
		self.speed = Twist()
		self.Flag_left =False
		self.Flag_right =False
		self.sub_gps = rospy.Subscriber('/fix', NavSatFix, self.GPS) #Subscriber to the topic /fix
		print(self.goal_x,self.goal_y)
		R = rospy.Rate(1)
		R.sleep()
		self.sub2 = rospy.Subscriber("/ebot/laser/scan",LaserScan,self.get_laser)
		self.sub = rospy.Subscriber('/imu', Imu, self.Angle)	     #Subscriber to the /imu topic
		self.pub = rospy.Publisher('/cmd_vel',Twist, queue_size = 1) #Publisher to /cmd_vel topic

	#Function to correct the angle and convert range from (-180,180) to (0,360):	
	def Angle_correction(self,az,angle):
		if(az<0):
			az_angle = 360+az
		else:
			az_angle = az
		if(angle<0):
			angle_conv = 360+angle
		else:
			angle_conv = angle

		return az_angle,angle_conv

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

	def GPS(self,req):
		self.latitude = req.latitude
		self.longitude = req.longitude
		self.altitude = req.altitude
		geodesic = pyproj.Geod(ellps='WGS84')
		self.fwd_angle,self.back_angle,self.dist = geodesic.inv(self.longitude,self.latitude,self.goal_y,self.goal_x)
		self.fwd_angle = -self.fwd_angle
		rospy.loginfo("Distance from goal: %lf",self.dist)
	
	#Function that sets the linear velocity of the rover to 0.2 when it is aligned and at a distance greater than 0.2 m (Forward Traversal):

	def Straight(self):
		Distance = self.dist
		rospy.loginfo("Distance from goal: %lf",Distance)
		if(Distance>0.1):
			self.speed.linear.x=0.5*self.dist/10
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
		fwd_net,angle_net = self.Angle_correction(self.fwd_angle,self.angle)
		#rospy.loginfo("IMU angle: %lf, Azimuth Angle: %lf",angle_net,fwd_net)
		theta_input = fwd_net-angle_net
		#rospy.loginfo("corrected angle diff: %lf",theta_input)
		self.theta = self.Short_Angle(theta_input)
		#rospy.loginfo("corrected shortest angle diff: %lf",self.theta)
		if(math.fabs((self.theta))>1.25):
			self.speed.angular.z=0.5*(self.fac*self.dist/10)
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
		if(self.dist>2):
			if(self.dist>2):
				if(self.obs[1] or self.obs[0] or self.obs[2]):
					self.avoid()
				elif(self.Flag_left or self.Flag_right):
					self.speed.linear.x = 0.5
					self.speed.angular.z = 0
					self.pub.publish(self.speed)
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
		else:
			self.speed.linear.x=0
			self.pub.publish(self.speed)
			print("Traversal Complete")
			rospy.signal_shutdown("Traversal Complete") #Shuts the node down once Traversal is complete

#		if(self.Flag):
#			self.Turn()
#		else:
#			print("Rover Aligned. Executing Traversal forward")
#			self.Straight()
	def avoid(self):
		print(self.obs)
		if(self.obs[0] and not self.obs[1] and not self.obs[2]): # TFF
			self.speed.linear.x = 0.5*self.dist/10
			self.speed.angular.z = 0.5*self.dist/10
			self.pub.publish(self.speed)
		elif(self.obs[1] and not self.obs[0] and not self.obs[2]): # FTF
			self.speed.linear.x = 0.5*self.dist/10
			self.speed.angular.z = 0.5
			self.pub.publish(self.speed)
		elif(self.obs[2] and not self.obs[1] and not self.obs[0]): # FFT
			self.speed.linear.x = 0.5*self.dist/10
			self.speed.angular.z = -0.5*self.dist/10
			self.pub.publish(self.speed)
		elif(self.obs[1] and self.obs[2] and not self.obs[0]): #FTT
			self.speed.linear.x = 0
			self.speed.angular.z = -0.5*self.dist/10
			self.pub.publish(self.speed)
		elif(self.obs[0] and self.obs[2] and not self.obs[1]): #TFT
			self.speed.linear.x = 0.5*self.dist/10
			self.speed.angular.z = 0
			self.pub.publish(self.speed)
		elif(self.obs[1] and self.obs[0] and not self.obs[2]): # TTF
			self.speed.linear.x = 0
			self.speed.angular.z = 0.5*self.dist/10
			self.pub.publish(self.speed)
		elif(self.obs[1] and self.obs[0] and self.obs[2]):	#TTT
			self.speed.linear.x = 0
			self.speed.angular.z = 0.5*self.dist/10
			self.pub.publish(self.speed)
		else:							#FFF
			self.speed.linear.x = 0.5*self.dist/10
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
			if(values[i]<0.75):
				self.obs.update({i : True})
			else:
				self.obs.update({i : False})
		#print(self.obs)
#Test Cases
#Goal 1
#latitude: 49.90005769387034
#longitude: 8.899823777341998

#Goal 2:
#latitude: 49.90008243488489
#longitude: 8.899816390105007

#Goal 3:
#latitude: 49.90008999971011
#longitude: 8.899966493145183

#Goal 4:
#latitude: 49.90011519129896
#longitude: 8.89997042805014

#Goal 5:
#latitude: 49.900112196783226
#longitude: 8.899844590253531

#Goal 6:
#latitude: 49.9000910927
#longitude: 8.89987401299

	
#Main function iterated repeatedly using rospy.spin() 

if __name__ == '__main__':
    rospy.init_node('IMU')
    x = np.float64(input("Enter goal x : "))	#Obtaining goal latitude from user
    y = np.float64(input("Enter goal y : ")) #Obtaining goal longitude from user
    Traverse(x,y) 				#Constructor of the class called
    rospy.spin()
