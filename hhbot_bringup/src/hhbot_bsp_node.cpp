#include "bsp/hhbot_bsp.h"

double RobotV_ = 0;
double YawRate_ = 0;

// 速度控制消息的回调函数
void cmdCallback(const geometry_msgs::Twist &msg)
{
    RobotV_ = msg.linear.x;
    YawRate_ = msg.angular.z;
}

int main(int argc, char **argv)
{
    // 初始化节点
    ros::init(argc, argv, "hhbot_bsp_node"); 
    ros::NodeHandle n;

    // 初始化HHbot
    hhbot_bsp::HHbot robot;
    if (!robot.init())
        ROS_ERROR("HHbot initialized failed.");
    ROS_INFO("HHbot initialized successful.");

    // 创建一个Subscriber，订阅名为cmd_vel的topic，注册回调函数cmdCallback
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 1000, cmdCallback);

    // 循环运行 50Hz
    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        robot.spinOnce(RobotV_, YawRate_);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
