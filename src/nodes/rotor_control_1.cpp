#include "rotor_control_1.h"

rotor_control::rotor_control()
:rotor_state_("stop") ,height_(1),yaw_(0),e0_(0),e1_(0),e2_(0),e3_(0)
,ox_(0),oy_(0),oz_(0),ow_(1)
{
    tf::Matrix3x3 trans_pose;
    
    tf_height_ = new Tf_Listerner("world","ardrone/base_link");
    dynamic_reconfigure::Server<my_test::Test_1Config> server;
    dynamic_reconfigure::Server<my_test::Test_1Config>::CallbackType f;
    f = boost::bind(&rotor_control::dynamicCB,this,_1,_2);
    sub_imu_ = n_.subscribe("/ardrone/imu", 1, &rotor_control::imuCallback,this);
    pub_speeds_ = n_.advertise<mav_msgs::Actuators>("/ardrone/command/motor_speed",1);
    server.setCallback(f);
    PthreadStart1();
    ros::Duration(5).sleep();
    while(ros::ok())
    {
        ros::spinOnce();
        //cout << "the rotor state is:" << rotor_state_ << endl;
        ros::Duration(0.1).sleep();
        double e_height,e_yaw,e0,e1,e2,e3;
        
        e_height = tf_height_->z() - height_;

        tf::Quaternion q1(ox_,oy_,oz_,ow_);
        trans_pose.setRotation(q1);
        double r_yaw,r_pitch,r_roll;
        trans_pose.getEulerYPR(r_yaw,r_pitch,r_roll);
        e_yaw = r_yaw - yaw_;
        if(e_yaw > 3.14159265) e_yaw = e_yaw - 2 * 3.14159265;
        else if(e_yaw < -3.14159265) e_yaw = 2 * 3.14159265 + e_yaw;
        cout << "r_yaw and yaw_:(" << r_yaw << "," << yaw_ << ")" << endl;

        e0 = trans_pose[2][0] * 0.12728 - trans_pose[2][1] * 0.12728 - e0_;
        e1 = trans_pose[2][0] * -0.12728 + trans_pose[2][1] * 0.12728 - e1_;
        e2 = trans_pose[2][0] * 0.12728 + trans_pose[2][1] * 0.12728 - e2_;
        e3 = trans_pose[2][0] * -0.12728 - trans_pose[2][1] * 0.12728 - e3_;

        pidControler(e_height,e_yaw,e0,e1,e2,e3);
    }
}

rotor_control::~rotor_control()
{}

void rotor_control::dynamicCB(my_test::Test_1Config& config,uint32_t level)
{
    ROS_INFO("Reconfigure Request: %d %d %d",config.kp,config.ki,config.kd);
    kp_h_ = config.kp;ki_h_ = config.ki;kd_h_ = config.kd;
    kp_yaw_ = config.kp_yaw;ki_yaw_ = config.ki_yaw;kd_yaw_ = config.kd_yaw;
    kp_tilt_ = config.kp_tilt;ki_tilt_ = config.ki_tilt;kd_tilt_ = config.kd_tilt;
}

void rotor_control::imuCallback(sensor_msgs::Imu msg)
{
    //double e_height,e_yaw,e0,e1,e2,e3;
    ow_ = msg.orientation.w;
    ox_ = msg.orientation.x;
    oy_ = msg.orientation.y;
    oz_ = msg.orientation.z;
    //pidControler(e_height,e_yaw,e0,e1,e2,e3);
}

void rotor_control::pidControler(double e_height,double e_yaw,double e0,double e1,double e2,double e3)
{
    static double t0 = 0;
    double t1;
    static double integral_eh = 0;
    static double integral_ey = 0;
    static double integral_e0 = 0;
    static double integral_e1 = 0;
    static double integral_e2 = 0;
    static double integral_e3 = 0;
    static double e_height0 = 0;
    static double e_yaw0 = 0;
    static double e00 = 0;
    static double e10 = 0;
    static double e20 = 0;
    static double e30 = 0;

    integral_eh += e_height;
    integral_ey += e_yaw;
    integral_e0 += e0;
    integral_e1 += e1;
    integral_e2 += e2;
    integral_e3 += e3;

    t1 = ros::Time::now().toSec();
    double det_t = t1 - t0;
    if(det_t == 0) det_t = 0.1;
    if(det_t > 0.2) det_t = 1;

    double pid_height = 0 - kp_h_ * e_height - ki_h_ * integral_eh * det_t - kd_h_ * (e_height - e_height0) / det_t;
    
    speed_command_.angular_velocities.resize(4);
    speed_command_.angular_velocities[0] = pid_height + kp_yaw_ * e_yaw + ki_yaw_ * integral_ey * det_t + kd_yaw_ * (e_yaw - e_yaw0) / det_t
                                         - kp_tilt_ * e0 - ki_tilt_ * integral_e0 * det_t - kd_tilt_ * (e0 - e00) / det_t;

    speed_command_.angular_velocities[1] = pid_height + kp_yaw_ * e_yaw + ki_yaw_ * integral_ey * det_t + kd_yaw_ * (e_yaw - e_yaw0) / det_t
                                         - kp_tilt_ * e1 - ki_tilt_ * integral_e1 * det_t - kd_tilt_ * (e1 - e10) / det_t;

    speed_command_.angular_velocities[2] = pid_height - kp_yaw_ * e_yaw - ki_yaw_ * integral_ey * det_t - kd_yaw_ * (e_yaw - e_yaw0) / det_t
                                         - kp_tilt_ * e2 - ki_tilt_ * integral_e2 * det_t - kd_tilt_ * (e2 - e20) / det_t;

    speed_command_.angular_velocities[3] = pid_height - kp_yaw_ * e_yaw - ki_yaw_ * integral_ey * det_t - kd_yaw_ * (e_yaw - e_yaw0) / det_t
                                         - kp_tilt_ * e3 - ki_tilt_ * integral_e3 * det_t - kd_tilt_ * (e3 - e30) / det_t;
    // cout << (int)speed_command_.angular_velocities[0] << ","
    //      << (int)speed_command_.angular_velocities[1] << ","
    //      << (int)speed_command_.angular_velocities[2] << ","
    //      << (int)speed_command_.angular_velocities[3] << endl;
    // cout << (int)(kp_h_ * e_height) << " , "
    //      << (int)(ki_h_ * integral_eh * det_t) << " , "
    //      << (int)(kd_h_ * (e_height - e_height0) / det_t) << " , "
    //      << (int)(kd_h_) << " , "
    //      << (int)(e_height - e_height0) << " , "
    //      << (det_t) << endl << endl;

    t0 = t1;
    e_height0 = e_height;
    e_yaw0 = e_yaw;
    e00 = e0;
    e10 = e1;
    e20 = e2;
    e30 = e3;
    pub_speeds_.publish(speed_command_);
}

string rotor_control::getch(void)
{
    struct termios oldattr,newattr;
    int ch;
    tcgetattr(STDIN_FILENO,&oldattr);
    newattr = oldattr;
    newattr.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO,TCSANOW,&newattr);
    ch = getchar();
    tcsetattr(STDIN_FILENO,TCSANOW,&oldattr);
    string St1(1,ch);
    return St1;
}

void rotor_control::PthreadStart1()
{
    if(pthread_create(&m_tid1_,NULL,ThreadStart1,(void*)this) != 0)
    {
        ROS_INFO("Start detector key_board thread failed!");
        return; 
    }
}

void* rotor_control::ThreadStart1(void* arg)
{
    rotor_control* ptr = (rotor_control*) arg;
    ptr->thread1Run();
    return NULL;
}

void rotor_control::thread1Run()
{
    cout << "start detector key board." << endl;
    static string key_char;
    //keyboard_timer_ = n_.createTimer(ros::Duration(1),&rotor_control::keyboardCb,this);
    while(ros::ok())
    {
        key_char = getch();
        //cout << "test:" << key_char[0] << endl;
        if(key_char[0] == 'w') 
        {
            rotor_state_ = "front";
            e0_ = -0.01;
            e1_ = 0.01;
            e2_ = -0.01;
            e3_ = 0.01;
        }
        else if(key_char[0] == 's') 
        {
            rotor_state_ = "behind";
            e0_ = 0.01;
            e1_ = -0.01;
            e2_ = 0.01;
            e3_ = -0.01;
        }
        else if(key_char[0] == 'a') 
        {
            rotor_state_ = "left";
            e0_ = 0.01;
            e1_ = -0.01;
            e2_ = -0.01;
            e3_ = 0.01;
        }
        else if(key_char[0] == 'd') 
        {
            rotor_state_ = "right";
            e0_ = -0.01;
            e1_ = 0.01;
            e2_ = 0.01;
            e3_ = -0.01;
        }
        else if(key_char[0] == '8') 
        {
            rotor_state_ = "up";
            height_ += 0.01;
        }
        else if(key_char[0] == '2') 
        {
            rotor_state_ = "down";
            height_ -= 0.01;
            if(height_ < 0.08) height_ = 0.08;
        }
        else if(key_char[0] == '4') 
        {
            rotor_state_ = "turn left";
            yaw_ += 0.03;
            if(yaw_ > 3.14159265) yaw_ = yaw_ - 2 * 3.14159265;
        }
        else if(key_char[0] == '6') 
        {
            rotor_state_ = "turn right";
            yaw_ -= 0.03;
            if(yaw_ < -3.14159265) yaw_ = yaw_ + 2 * 3.14159265;
        }
        else 
        {
            rotor_state_ = "stop";
            e0_ = 0;
            e1_ = 0;
            e2_ = 0;
            e3_ = 0;
        }
        //cout << rotor_state_ << endl;
    }
}

void rotor_control::keyboardCb(const ros::TimerEvent&)
{
    //cout << "test." << endl;
    //rotor_state_ = "stop";
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "rotor_control");
    cout << "this is a program of control system of rotor." << endl;
    rotor_control rotor_control_test;
    return 0;
}
