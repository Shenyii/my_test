#ifndef ROTOR_CONTROL_1_H
#define ROTOR_CONTROL_1_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include "geometry_msgs/Twist.h"
#include "mav_msgs/Actuators.h"
#include <iostream>
#include <sstream>
#include <string>
#include "tf_listerner.h"
#include <dynamic_reconfigure/server.h>
#include <my_test/Test_1Config.h>
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

using namespace std;

class rotor_control
{
public:
    rotor_control();
    ~rotor_control();
    void start();

private:
    string rotor_state_;   //up,down,left,right,front,behind,stop
    ros::NodeHandle n_;
    ros::Subscriber sub_imu_;
    ros::Publisher pub_speeds_;// = nh.advertise<std_msgs::String>("topic_name", 1000);
    mav_msgs::Actuators speed_command_;
    ros::Timer keyboard_timer_;
    void keyboardCb(const ros::TimerEvent&);
    void imuCallback(sensor_msgs::Imu msg);
    void pidControler(double e_height,double e_yaw,double e0,double e1,double e2,double e3);

    int kp_h_,ki_h_,kd_h_;
    int kp_yaw_,ki_yaw_,kd_yaw_;
    int kp_tilt_,ki_tilt_,kd_tilt_;
    double height_;
    double yaw_;
    double e0_,e1_,e2_,e3_;
    double ow_,ox_,oy_,oz_;
    Tf_Listerner* tf_height_;
    void dynamicCB(my_test::Test_1Config& config,uint32_t level);

    string getch(void);
    void PthreadStart1();
    static void* ThreadStart1(void * arg);
    void thread1Run();
    pthread_t m_tid1_;
};

#endif