/***
    Author: Samruddhi Kadam
    Course: Udemy:ROS for Beginners
    Project 1 - Turtlesim Cleaner application
    Taught by: Anis Koubaa
***/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
//create publisher to publish Twist message
ros::Publisher velocity_publisher;

using namespace std;
const double PI = 3.14159265359;
//method to move the robot straight
//declaring move method

void move(double speed, double distance, bool isForward);
void rotate(double angular_speed, double angle, bool clockwise);
double degrees2radians(double angle_in_degrees);

int main(int argc, char **argv)
{
    //Initialize new ROS node named "robot_cleaner"
    ros::init(argc, argv, "robot_cleaner");
    ros::NodeHandle n;
    
    double speed;
    double distance;
    bool isForward;

    double angular_speed;
    double angle;
    bool clockwise;

    //Initialize publisher
    velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",10);

    //To test the publisher
    cout << " Enter speed: ";
    cin >> speed;
    cout << " Enter distance: ";
    cin >> distance;
    cout << " Move forward?: ";
    cin >> isForward;
    move(speed, distance, isForward);

    cout << "Enter angular velocity (degrees/sec): ";
    cin >> angular_speed;
    cout << "Enter desired angle (degrees): ";
    cin >> angle;
    cout << "Clockwise ?: ";
    cin >> clockwise;
    rotate(degrees2radians(angular_speed), degrees2radians(angle),clockwise);




    //move(2.0,5.0,1);
}

/** 
 * makes the robot move with a certain linear velocity for a
 * certain distance in a forward or backward straight direction
**/
void move(double speed, double distance, bool isForward){
    //create geometry_msgs/Twist message
    geometry_msgs::Twist vel_msg;
    //distance = speed * time
    //set a random linear velocity in x-axis
    if (isForward)
    {
        vel_msg.linear.x = abs(speed);
    }
    else
    {
        vel_msg.linear.x = -abs(speed);
    }
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;

    //set a random angular velocity in z-axis
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;

    //t0:current time
    double t0 = ros::Time::now().toSec();
    double curr_distance = 0;
    ros::Rate loop_rate(100);
    //loop
    //publish Twist message (i.e velocity)
    //estimate the distance by speed* (t1-t0)
    // current distance moved by robot
    do{
        velocity_publisher.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        curr_distance = speed * (t1-t0);
        ros::spinOnce();
        loop_rate.sleep();

    }while(curr_distance < distance);
    vel_msg.linear.x = 0;
    velocity_publisher.publish(vel_msg);
    
}

void rotate(double angular_speed, double relative_angle, bool clockwise){

    geometry_msgs::Twist vel_msg;
    //set a random linear velocity in the x-axis
    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    //set a random angular velocity in y-axis
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    
    if(clockwise)
    {
        vel_msg.angular.z = -abs(angular_speed);
    }  
    else
    {
        vel_msg.angular.z = abs(angular_speed);
    }   

    //initializing variables:
    double current_angle = 0.0;
    double t0 = ros::Time::now().toSec();
    ros::Rate loop_rate(10);

    do{
        velocity_publisher.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        current_angle = angular_speed * (t1-t0);
        ros::spinOnce();
        loop_rate.sleep();
    }while(current_angle<relative_angle);

    vel_msg.angular.z = 0;
    velocity_publisher.publish(vel_msg);
    
}

double degrees2radians(double angle_in_degrees){
    return angle_in_degrees *PI /180.0;
}


