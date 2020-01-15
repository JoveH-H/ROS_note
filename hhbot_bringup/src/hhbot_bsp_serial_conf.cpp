#include <vector>
#include <serial/serial.h> // ROS内置串口功能包
#include "bsp/hhbot_bsp.h"

namespace hhbot_bsp
{
  const unsigned char header[2] = {0x88, 0x88};   // 帧头
  const unsigned char ender[2] = {0x66, 0x66};    // 帧尾
  const double ROBOT_RADIUS = 0.025; // 车轮的半径
  const double ROBOT_LENGTH = 0.140; // 车轮的距离
  serial::Serial ser;                // 声明串口对象
  using namespace boost::asio;       // 定义一个命名空间，用于后面的读写操作
  boost::asio::io_service iosev;

  // 姿势协方差
  boost::array<double, 36> odom_pose_covariance = {
    {1e-3, 0, 0, 0, 0, 0,
     0, 1e-3, 0, 0, 0, 0,
     0, 0, 1e6, 0, 0, 0,
     0, 0, 0, 1e6, 0, 0,
     0, 0, 0, 0, 1e6, 0,
     0, 0, 0, 0, 0, 1e-3}};

  // 转动协方差
  boost::array<double, 36> odom_twist_covariance = {
    {1e-3, 0, 0, 0, 0, 0,
     0, 1e-3, 0, 0, 0, 0,
     0, 0, 1e6, 0, 0, 0,
     0, 0, 0, 1e6, 0, 0,
     0, 0, 0, 0, 1e6, 0,
     0, 0, 0, 0, 0, 1e-3}};

  // 声明里程联合体
  union odometry {
    float odoemtry_float;
    unsigned char odometry_char[4];
  } vel_left, vel_right;

  // 声明车轮驱动发送数据联合体
  union sendData {
    float data_float;
    unsigned char data_char[4];
  } leftdata, rightdata;

  // 初始化变量
  HHbot::HHbot() : x_(0.0), y_(0.0), th_(0.0),
                  vx_(0.0), vy_(0.0), vth_(0.0)
  {
  }

  HHbot::~HHbot()
  {
  }

  bool HHbot::init()
  {
    // 默认配置端口hhbot_bsp（已绑定），波特率115200，连接超时0.01秒
    ser.setPort("/dev/hhbot_bsp");
    ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(10);
    ser.setTimeout(to);

    // 打开串口
    ser.open();

    // 检测串口是否已经打开，并给出提示信息
    if (ser.isOpen())
    {
      ROS_INFO_STREAM("BSP serial init ok");
    }
    else
    {
      ROS_ERROR("BSP serial init error");
      return -1;
    }

    ros::Time::init();
    current_time_ = ros::Time::now();
    last_time_ = ros::Time::now();

    // 定义发布消息的名称
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 500);

    return true;
  }

  bool HHbot::readSpeed()
  {
    int i, length = 13;
    unsigned char checkSum;
    unsigned char buf[13] = {0};

    // 读取缓存区长度
    if (ser.available() >= length)  
    {
      // 读取串口数据
      ser.read(buf, length);
      
      // 清空缓存区
      ser.flushInput();
      
      // 检查信息头
      if (buf[0] != header[0] || buf[1] != header[1])
      {
        ROS_ERROR("Received message header error!");
        return false;
      }

      // 检查信息尾
      if (buf[length - 2] != ender[0] || buf[length - 1] != ender[1])
      {
        ROS_ERROR("Received message ender error!");
        return false;
      }

      // 转换校验
      checkSum = getCrc8(buf, length - 3);

      // 检查信息校验值
      if (checkSum != buf[length - 3])
      {
        ROS_ERROR("Received data check sum error!");
        return false;
      }

      // 读取速度值
      for (i = 0; i < 4; i++)
      {
        vel_left.odometry_char[i] = buf[i + 2];
        vel_right.odometry_char[i] = buf[i + 6];
      }

      ROS_INFO("Read odoemtry:%f,%f", vel_left.odoemtry_float, vel_right.odoemtry_float);

      // 积分计算里程计信息
      vx_ = (vel_right.odoemtry_float + vel_left.odoemtry_float) / 2;
      vth_ = (vel_right.odoemtry_float - vel_left.odoemtry_float) / ROBOT_LENGTH;

      // 记录实时时间戳
      ros::Time curr_time;
      curr_time = ros::Time::now();

      // 参数运算
      double dt = (curr_time - last_time_).toSec();
      double delta_x = (vx_ * cos(th_) - vy_ * sin(th_)) * dt;
      double delta_y = (vx_ * sin(th_) + vy_ * cos(th_)) * dt;
      double delta_th = vth_ * dt;

      x_ += delta_x;
      y_ += delta_y;
      th_ += delta_th;
      last_time_ = curr_time;
    }
    return true;
  }

  void HHbot::writeSpeed(double RobotV, double YawRate)
  {
    int i, length = 13;
    unsigned char buf[13] = {0};
    
    // 计算左右轮期望速度
    if (RobotV == 0)
    {
      leftdata.data_float = -YawRate * ROBOT_LENGTH / 2.0;
      rightdata.data_float = -leftdata.d;
    }
    else if (YawRate == 0)
    {
      leftdata.data_float = RobotV;
      rightdata.data_float = RobotV;
    }
    else
    {
      leftdata.data_float = RobotV - YawRate * ROBOT_LENGTH / 2.0;
      rightdata.data_float = RobotV + YawRate * ROBOT_LENGTH / 2.0;
    }

    // 设置帧头
    for (i = 0; i < 2; i++)
      buf[i] = header[i];

    // 设置数据
    for (i = 0; i < 4; i++)
    {
      buf[i + 2] = leftdata.data_char[i];
      buf[i + 6] = rightdata.data_char[i];
    }

    // 转换校验
    buf[10] = getCrc8(buf, length - 3);

    // 设置帧尾
    for (i = 0; i < 2; i++)
      buf[11 + i] = ender[i];

    // 通过串口下发数据
    ser.write(buf, 13);
  }

  bool HHbot::spinOnce(double RobotV, double YawRate)
  {
    // 下发机器人期望速度
    writeSpeed(RobotV, YawRate);

    // 读取机器人实际速度
    readSpeed();

    // 记录实时时间戳
    current_time_ = ros::Time::now();

    // 发布TF
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
    geometry_msgs::Quaternion odom_quat;
    odom_quat = tf::createQuaternionMsgFromYaw(th_);
    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster_.sendTransform(odom_trans);

    // 发布里程计消息
    nav_msgs::Odometry msgl;
    msgl.header.stamp = current_time_;
    msgl.header.frame_id = "odom";
    msgl.pose.pose.position.x = x_;
    msgl.pose.pose.position.y = y_;
    msgl.pose.pose.position.z = 0.0;
    msgl.pose.pose.orientation = odom_quat;
    msgl.pose.covariance = odom_pose_covariance;
    msgl.child_frame_id = "base_footprint";
    msgl.twist.twist.linear.x = vx_;
    msgl.twist.twist.linear.y = vy_;
    msgl.twist.twist.angular.z = vth_;
    msgl.twist.covariance = odom_twist_covariance;
    odom_pub.publish(msgl);
  }

  // 转换校验函数
  unsigned char HHbot::getCrc8(unsigned char *ptr, unsigned short len)
  {
    unsigned char crc;
    unsigned char i;
    crc = 0;
    while (len--)
    {
      crc ^= *ptr++;
      for (i = 0; i < 8; i++)
      {
        if (crc & 0x01)
        {
          crc = (crc >> 1) ^ 0x8C;
        }
        else
          crc >>= 1;
      }
    }
    return crc;
  }

} // namespace hhbot_bsp
