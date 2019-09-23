#ifndef KEY_CONTROL_TEST1_H
#define KEY_CONTROL_TEST1_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include "geometry_msgs/Twist.h"
#include "mav_msgs/Actuators.h"

int speed_bias;

void subCallback(geometry_msgs::Twist msg)
{
    if(msg.linear.x > 0)
    {
        speed_bias += 3;
    }
    else
    {
        speed_bias -= 3;
    }
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "key_control_test1");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<mav_msgs::Actuators>("/ardrone/command/motor_speed", 1);
    ros::Subscriber sub = nh.subscribe("/turtle1/cmd_vel", 1, subCallback);
    speed_bias = 0;
    mav_msgs::Actuators pub_msg;
    while(ros::ok())
    {
        pub_msg.angular_velocities.resize(4);
        pub_msg.angular_velocities[0] = 650 + speed_bias;
        pub_msg.angular_velocities[1] = 650 + speed_bias;
        pub_msg.angular_velocities[2] = 650 + speed_bias;
        pub_msg.angular_velocities[3] = 650 + speed_bias;
        std::cout << "the speed is:" << 650 + speed_bias << std::endl;
        pub.publish(pub_msg);
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    return 0;
}

#endif
