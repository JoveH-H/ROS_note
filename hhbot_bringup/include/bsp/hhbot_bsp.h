#ifndef HHBOT_BSP_H
#define HHBOT_BSP_H

#include <ros/ros.h> // 包含ROS的头文件
#include <ros/time.h>
#include <string>
#include <math.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <boost/asio.hpp> //包含boost库函数
#include <boost/bind.hpp>
#include "std_msgs/String.h" //ros定义的String数据类型

namespace hhbot_bsp
{
class HHbot
{
public:
  HHbot();
  ~HHbot();
  bool init();
  bool spinOnce(double RobotV, double YawRate);

private:
  bool readSpeed();
  void writeSpeed(double RobotV, double YawRate);
  unsigned char getCrc8(unsigned char *ptr, unsigned short len);

private:
  ros::Time current_time_, last_time_;

  double x_;
  double y_;
  double th_;

  double vx_;
  double vy_;
  double vth_;

  ros::NodeHandle n;
  ros::Publisher odom_pub;
  tf::TransformBroadcaster odom_broadcaster_;
};

} // namespace hhbot_bsp

#endif /* HHbot_BSP */
