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


class Traverse:

	#The init function executed every time the constructor/Class name is called in main:

	def __init__(self,x,y):
		self.goal_x = x
		self.goal_y = y
		self.obs = {}
		self.speed = Twist()
		self.pub = rospy.Publisher('/cmd_vel',Twist, queue_size = 1) #Publisher to /cmd_vel topic
		self.sub2 = rospy.Subscriber("/ebot/laser/scan",LaserScan,self.get_laser)
		self.sub = rospy.Subscriber('/imu', Imu, self.Angle)	
		self.sub_gps = rospy.Subscriber('/odom', Odometry, self.Odom) #Subscriber to the topic /fix
		print("Goal Recieved : "+str(self.goal_x) +" , " + str(self.goal_y))
		R = rospy.Rate(1)
		R.sleep()  


	def get_laser(self,msg):
		tot = 720
		fac = 3
		Iter = tot/fac
		Range = 0
		max_range = msg.range_max
		self.speed = Twist()
		values =[]
		for i in range(0,fac):
			values.append(min(min(msg.ranges[int(Range):int(Range+Iter)]),max_range))
			Range+=Iter
		self.obs = {}
		for i in range(0,fac):
			if(values[i]<1):
				self.obs.update({i : True})
			else:
				self.obs.update({i : False})
		print(self.obs)
				

	def avoid(self):
		if(self.obs[0] and not self.obs[1] and not self.obs[2]): # TFF
			self.speed.linear.x = 0.5*self.dist/10
			self.speed.angular.z = 0.5*self.dist/10
			self.pub.publish(self.speed)
		elif(self.obs[1] and not self.obs[0] and not self.obs[2]): # FTF
			self.speed.linear.x = 0#0.5*self.dist/10
			self.speed.angular.z = 0.5*self.dist/10
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
			self.speed.angular.z = 0.5*self.fac
			self.pub.publish(self.speed)
		else:							#FFF
			self.speed.linear.x = 0.5*self.fac
			self.speed.angular.z = 0
			self.pub.publish(self.speed)
			
							
	def Angle(self,msg):
		self.x = msg.orientation.x
		self.y = msg.orientation.y
		self.z = msg.orientation.z
		self.w = msg.orientation.w
		(self.roll, self.pitch, self.yaw) = euler_from_quaternion([self.x,self.y,self.z,self.w])
		self.yaw = self.Angle_correction(math.degrees(self.yaw))
		rospy.loginfo("yaw : %lf",self.yaw)

	def Odom(self,req):
		self.x_co = req.pose.pose.position.x
		self.y_co = req.pose.pose.position.y
		self.z_co = req.pose.pose.position.z
		self.diff_x = self.goal_x-self.x_co
		self.diff_y = self.goal_y-self.y_co
		self.dist = math.sqrt((self.diff_x)**2+(self.diff_y)**2)
		rospy.loginfo("Distance from goal: %lf",self.dist)
		if(self.goal_x>0 and self.goal_y>0):
			self.angle_goal = math.fabs((math.degrees((math.atan(self.goal_y/self.goal_x)))))
		elif(self.goal_x>0 and self.goal_y<0):
			self.angle_goal = (270+math.fabs((math.degrees((math.atan(self.goal_x/self.goal_y))))))
		elif(self.goal_x<0 and self.goal_y<0):
			self.angle_goal = 180+math.fabs((math.degrees((math.atan(self.goal_y/self.goal_x)))))
		else:
			self.angle_goal = 90+math.fabs((math.degrees((math.atan(self.goal_x/self.goal_y)))))
		rospy.loginfo("Angle to goal : %lf",self.angle_goal)
		self.angle_diff = self.angle_goal - self.yaw
		self.angle_diff = self.Short_Angle(self.angle_diff)
		if(self.dist>1):
			if(self.obs[1] or self.obs[0] or self.obs[2]):
				rospy.loginfo("Obstacle Hist : "+str(self.obs))
				self.avoid()
			else:
				self.Turn()
		else:
			self.speed = Twist()
			self.pub.publish(self.speed)
			print("Traversal Complete")
			rospy.signal_shutdown("Traversal Complete")
	
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

	
	def Angle_correction(self,angle):
		if(angle<0):
			angle_conv = 360+angle
		else:
			angle_conv = angle

		return angle_conv		
		
	def Turn(self):
		rospy.loginfo("Angle difference : %lf",self.angle_diff)
		if(math.fabs((self.angle_diff))>1):
			rospy.loginfo("Aligning Bot")
			self.speed.linear.x=0.0
			self.speed.angular.z=self.dist/10*(self.fac)
			self.pub.publish(self.speed)
		else:
			self.speed.angular.z=0.0
			self.pub.publish(self.speed)
			print("Bot Aligned")
			self.Straight()
			#self.Flag = False
		
	
	def Straight(self):
		if(self.dist>1):
			self.speed.linear.x=self.dist/10
			self.pub.publish(self.speed)
		else:
			self.speed = Twist()
			self.pub.publish(self.speed)
			print("Traversal Complete")
			rospy.signal_shutdown("Traversal Complete") #Shuts the node down once Traversal is complete
			

		


#Test Cases

"""
Goal 1:
x:  -1.3748298229342164
y: 7.5845298605847

Goal 2:
x: 0.8046492983061181
y: 5.52346121544775

Goal 3:
x: 0.7497328248690106
y: -2.250622791091676

Goal 4:
x: 2.658286318849095
y: -1.8904649525831771

Goal 5:
x: 3.0807281616358453
y: 7.443939927195515
"""
#Main function iterated repeatedly using rospy.spin() 

if __name__ == '__main__':
    rospy.init_node('IMU')
    x = np.float64(input("Enter goal x : "))	#Obtaining goal latitude from user
    y = np.float64(input("Enter goal y : ")) #Obtaining goal longitude from user
    Traverse(x,y) 				#Constructor of the class called
    rospy.spin()
