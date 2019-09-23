#ifndef MY_PROGRAM_H
#define MY_PROGRAM1_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>


int main(int argc,char** argv)
{
    ros::init(argc, argv, "tf_test1");
    ros::NodeHandle nh;
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    double roll = 0,pitch = 0,yaw = 0;
    while(ros::ok())
    {
        yaw += 0.1;
        q.setRPY(roll,pitch,yaw);
        transform.setOrigin(tf::Vector3(0.1862,0.1075,0.037));
        transform.setRotation(q);
        std::cout<<"send tf transform function"<<std::endl;
        br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"firefly/base_link","firefly/rotor_0"));
        std::cout << "w=" << q[3] << ",x=" << q[0] << ",y=" << q[1] << ",z=" << q[2] << std::endl;
        ros::Duration(1).sleep();
        ros::spinOnce();
    }
    return 0;
}

#endif
