#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import NavSatFix, Imu, LaserScan
import pyproj
import numpy as np


#first quad
# 49.9000802426
# 8.89992844565

#third quad
# 49.8999726435
# 8.90010670204

#second quad
# 49.8999348482
# 8.8999248023

#fourth quad
# 49.9000773681
# 8.90014574245


class ATT():
    
    #######CONSTRUCTOR#######
    def __init__(self,lat,long):
        self.end_lat = lat
        self.end_long = long
        self.start_lat, self.start_long, self.quat, self.e1, self.yaw = 0, 0, 0, 0, 0
        self.state_description, self.linear_x, self.angular_z = '', 0, 0
        self.regions = 0
        self.g, self.az12, self.az21, self.a, self.az, self.dist = 0, 0, 0, 0, 0, 0.55
        self.error, self.d = 0, 0

        rospy.Subscriber('/fix', NavSatFix, self.callback_gps)
        rospy.Subscriber('/imu', Imu, self.callback_imu)#/group_task2/imu
        rospy.Subscriber('/ebot/laser/scan', LaserScan, self.clbk_laser)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(1)
        self.vel = Twist()
        rospy.spin()
    
    #######GPS CALLBACK#######
    def callback_gps(self, msg):
        # self.return_val = msg
        self.start_lat, self.start_long = msg.latitude, msg.longitude
        ## print ("starting lat = ", self.start_lat, "starting long = ", self.start_long)
        # print yaw
        # print msgt
        self.pyproj_get_data()

    #######IMU CALLBACK#######    
    def callback_imu(self, msg):
        self.quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.e1 = euler_from_quaternion(self.quat)
        self.yaw = math.degrees(self.e1[2])  
        ## print ("yaw = ", self.yaw)
        
        self.rotation()


    #######LASER SCANNER CALLBACK######  
    def clbk_laser(self, msg):
        self.regions = {
        'right':  min(min(msg.ranges[0:14]), 10),
        'fright': min(min(msg.ranges[14:309]), 10),
        'front':  min(min(msg.ranges[310:410]), 10),
        'fleft':  min(min(msg.ranges[411:703]), 10),
        'left':   min(min(msg.ranges[704:719]), 10),
        }
        # self.take_action()

    #######LIDAR SPLICING OBS AVD#######
    def take_action(self):
    
        self.d = 1.5
        if self.regions['front'] > self.d and self.regions['fleft'] > self.d and self.regions['fright'] > self.d:
            self.state_description = 'case 1 - nothing'
            self.linear_x = 1
            self.angular_z = 0

        elif self.regions['front'] < self.d and self.regions['fleft'] > self.d and self.regions['fright'] > self.d:
            self.state_description = 'case 2 - front'
            self.linear_x = 1
            self.angular_z = -0.5

        elif self.regions['front'] > self.d and self.regions['fleft'] > self.d and self.regions['fright'] < self.d:
            self.state_description = 'case 3 - fright'
            self.linear_x = 1
            self.angular_z = -0.5

        elif self.regions['front'] > self.d and self.regions['fleft'] < self.d and self.regions['fright'] > self.d:
            self.state_description = 'case 4 - fleft'
            self.linear_x = 1
            self.angular_z = 0.5

        elif self.regions['front'] < self.d and self.regions['fleft'] > self.d and self.regions['fright'] < self.d:
            self.state_description = 'case 5 - front and fright'
            self.linear_x = 1
            self.angular_z = -0.5

        elif self.regions['front'] < self.d and self.regions['fleft'] < self.d and self.regions['fright'] > self.d:
            self.state_description = 'case 6 - front and fleft'
            self.linear_x = 1
            self.angular_z = 0.5

        elif self.regions['front'] > self.d and self.regions['fleft'] < self.d and self.regions['fright'] < self.d:
            self.state_description = 'case 8 - fleft and fright'
            self.linear_x = 1
            self.angular_z = 0

        elif self.regions['front'] < self.d and self.regions['fleft'] < self.d and self.regions['fright'] < self.d:
            self.state_description = 'case 7 - front and fleft and fright'
            self.linear_x = 0
            self.angular_z = 0.5

        # elif self.regions['right'] < self.d:
        #     self.state_description = 'case 9 - wall to right'
        #     self.linear_x = 0.6
        #     self.angular_z = 0

        # elif self.regions['left'] < self.d:
        #     self.state_description = 'case 10 - wall to left'
        #     self.linear_x = 0.6
        #     self.angular_z = 0

        else:
            self.state_description = 'unknown case'
            rospy.loginfo(self.regions)

        rospy.loginfo(self.state_description)
        self.vel.linear.x = self.linear_x
        self.vel.angular.z = self.angular_z
        
        if self.dist > 0.5:
            self.pub.publish(self.vel)

        else:
            self.vel.linear.x = 0
            self.vel.angular.z = 0
            self.pub.publish(self.vel)
            rospy.signal_shutdown('done')

       
    #######PYPROJ PACKAGE FUCNTION CALL#######   
    def pyproj_get_data(self):
        self.g = pyproj.Geod(ellps='WGS84')
        (self.az12, self.az21, self.dist) = self.g.inv(self.start_long, self.start_lat, self.end_long, self.end_lat)
        self.a = [self.az12, self.az21, self.dist]
        # print self.a
        self.az = -self.a[0]
        self.dist = self.a[2]
        print ("azimuth = ", self.az, "disance = ", self.dist)

    #######ALIGNMENT FIXING FUNCTION CALL#######       
    def rotation(self):
        

        if self.yaw < 0:
            self.yaw = self.yaw + 360

        if self.az < 0:
            self.az = self.az + 360

        self.error = self.az - self.yaw


        if self.error < 0:
            if math.fabs(self.error) > 180:
                self.error = self.error + 360
                self.vel.angular.z = -0.5  #self.error / 100  #0.8
            else:
                self.vel.angular.z =  0.5  #self.error / 100
        else:
            if self.error > 180:
                self.error = 360 - self.error
                self.vel.angular.z =  0.5 #self.error / 100
            else:
                self.vel.angular.z = -0.5 #self.error / 100

        if math.fabs(self.error) > 0.9:
            self.pub.publish(self.vel)
            print('target and current : ', self.az, self.yaw)
            if self.regions['front'] < 1 or self.regions['fright'] < 1 or self.regions['fleft'] < 1:
                self.take_action()

        else:    
            self.vel.angular.z = 0
            self.pub.publish(self.vel) 
            # self.linear_movtement()
            self.take_action()
            self.rate.sleep()
   
    #######DISTANCE MOVEMENT FUNCTION#######
    # def linear_movement(self):
    #     self.vel.linear.x = 0.25

        # if self.dist > 0.5:
        #     self.pub.publish(self.vel)

        # else:
        #     self.vel.linear.x = 0
        #     self.pub.publish(self.vel)
        #     rospy.signal_shutdown('done')
            

#######MAIN FUNCTION#######
def main():
    rospy.init_node('att', anonymous = True)
    x = np.float64(input("Enter Latitude : "))	#Obtaining goal latitude from user
    y = np.float64(input("Enter Longitude : ")) 
    ATT(x,y)
    
    
if __name__ == '__main__':
    main()
