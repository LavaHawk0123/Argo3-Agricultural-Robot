
#include "iostream"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "math.h"
#include "tf/transform_datatypes.h"
#include <string>
#include <boost/geometry.hpp>

#define SEMI_CIRCLE 180
#define CIRCLE 360
#define EARTH_RADIUS 6371

namespace gp
{
    class globalPlanner
    {
        private:

            // Velocity Publisher
            ros::Publisher pub_vel_;

            //Azimuth Publisher
            ros::Publisher pub_az_;

            // GPS Subscriber
            ros::Subscriber sub_gps_;

            // IMU Subscriber
            ros::Subscriber sub_imu_;

            /*1st Latitude variable 
              The goal latitude co-ordinate            
            */
            double goal_lat_;

            /*1st Longitude variable 
              The goal longitude co-ordinate            
            */
            double goal_long_;
            
            /*2nd Latitude variable 
              The latitude co-ordinates recieved form the sensor            
            */
            double sensor_lat_;
            
            /*2nd Longitude variable 
              The longitude co-ordinates recieved form the sensor
            */
            double sensor_long_;

            //current yaw of the rover
            double yaw_;

            //Forward azimuthal angle(bearing)
            double error_angle_;
            
            //Distance between rover and goal
            double error_distance_;
            
            //Initial Distance
            double in_error_distance_;

            //Velocity to be published
            geometry_msgs::Twist vel_;

            //Difference angle between rover and goal
            double diff_angle_;  

            //Maximum linear Velocity of the rover
            float linear_max_;

            //Minimum linear Velocity of the rover
            float linear_min_;

            //Maximum angular Velocity of the rover
            float ang_max_;

            //Minimum angular Velocity of the rover
            float ang_min_;

            //Distance threshold
            float dist_thres_;

            //Angle threshold
            float ang_thres_;

            typedef struct pid
            {
                float kp;
                float kd;
            }pid_;

            //Node Handle
            ros::NodeHandle nh; 


        public:
            //Class Constructor
            globalPlanner(); 
            //Gps Callback function                     
            void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
            //IMU Callback function
            void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
            //Distance and Angle Calculator 
            void distance();
            //Align the rover
            void rotation();
            //Straight motion to the Goal
            void linearMovement();
            //Caller function 
            void caller();
    };
}


