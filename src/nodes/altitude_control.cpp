#ifndef ALTITUDE_CONTROL_H
#define ALTITUDE_CONTROL_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include "geometry_msgs/Twist.h"
#include "mav_msgs/Actuators.h"
#include <iostream>
#include "tf_listerner.h"
#include <dynamic_reconfigure/server.h>
#include <my_test/Test_1Config.h>

int speed_g = 0;
double altitude_g = 0.08;
int Kp = 430;
int Ki = 86;
int Kd = 118;
double M = 0.1;

void callbackDynamic(my_test::Test_1Config& config, uint32_t level)
{
	ROS_INFO("Reconfigure Request: %d %d %d",config.kp,config.ki,config.kd);
    Kp = config.kp;
    Ki = config.ki;
    Kd = config.kd;
}

void* threadAltitudeSet(void*)
{
    while(ros::ok())
    {
        std::cout << "the anticipate altitude is:";
        std::cin >> altitude_g;
        std::cout << std::endl;
    }
}

void pthreadAltitudeSet()
{
    pthread_t pid_usart;
    std::cout << "start set altitude" << std::endl;
    int pthread_spin = pthread_create(&pid_usart,NULL,threadAltitudeSet,NULL);
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "key_control_test1");
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<my_test::Test_1Config> server;
	dynamic_reconfigure::Server<my_test::Test_1Config>::CallbackType f;
	f = boost::bind(&callbackDynamic, _1, _2);
	server.setCallback(f);

    ros::Publisher pub = nh.advertise<mav_msgs::Actuators>("/ardrone/command/motor_speed", 1);
    mav_msgs::Actuators pub_msg;
    pub_msg.angular_velocities.resize(4);
    pthreadAltitudeSet();
    Tf_Listerner tf_world_to_base("world","ardrone/base_link");
    double sum_e = 0;
    double e0 = 0;
    double e1;
    while(ros::ok())
    {
        //std::cout << tf_world_to_base.x() << "," << tf_world_to_base.y() << "," << tf_world_to_base.z() << std::endl;
        double e = altitude_g - tf_world_to_base.z();
        sum_e += e;
        speed_g = Kp * e + Ki * sum_e * 0.1 + Kd * (e - e0) / 0.1;
        e0 = e;
        std::cout << "the bias is:" << e << "," << speed_g << std::endl;
        
        pub_msg.angular_velocities[0] = speed_g;
        pub_msg.angular_velocities[1] = speed_g;
        pub_msg.angular_velocities[2] = speed_g;
        pub_msg.angular_velocities[3] = speed_g;
        pub.publish(pub_msg);
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    return 0;
}

#endif
