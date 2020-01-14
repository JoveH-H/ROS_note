#!/usr/bin/env python
# coding=utf-8

import rospy  # 导入rospy模块 ROS的python客户端
from std_msgs.msg import String  # 导入std_msgs.msg模块的String消息类型
from nav_msgs.msg import Odometry  # 导入nav_msgs.msg模块的Odometry消息类型
from geometry_msgs.msg import Twist  # 导入geometry_msgs.msg模块的Twist消息类型
import hhbot_bsp_serial as bsp_serial  # 导入hhbot_bsp_serial模块 串口配置
import struct  # 导入struct模块 数据类型转换
import math  # 导入math模块 数据运算
import tf  # 导入tf模块 坐标转换

leftdata, rightdata = 0.0, 0.0  # 车轮驱动发送数据
vel_left, vel_right = 0.0, 0.0  # 车轮速度读取数据
ROBOT_RADIUS = 0.025  # 车轮的半径
ROBOT_LENGTH = 0.140  # 车轮的距离
last_time_ = 0.0  # 计算使用的时间戳
x_, y_, th_, vx_, vy_, vth_ = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0  # 参数运算数值
RobotV_, YawRate_ = 0.0, 0.0  # 读取运动控制数值

# 姿势协方差
odom_pose_covariance = (
    1e-3, 0, 0, 0, 0, 0,
    0, 1e-3, 0, 0, 0, 0,
    0, 0, 1e6, 0,0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e-3)

# 转动协方差
odom_twist_covariance = (
    1e-3, 0, 0, 0, 0, 0,
    0, 1e-3, 0, 0, 0, 0,
    0, 0, 1e6, 0,0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e-3)


def getCrc8(ptr1, len):
    """
    转换校验函数
    """
    crc = 0x00
    count = 0
    for ptr in ptr1:
        crc = crc ^ ord(ptr)
        for i in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc = crc >> 1
        count = count + 1
        if count >= len:
            break
    return crc


def SerialWrite(ser, RobotV, YawRate):
    """
    串口通信数据发送函数
    """
    global leftdata, rightdata

    # 计算左右轮期望速度
    if RobotV == 0:
        leftdata = -YawRate * ROBOT_RADIUS
        rightdata = YawRate * ROBOT_RADIUS
    elif YawRate == 0:
        leftdata = RobotV
        rightdata = RobotV
    else:
        r = RobotV / YawRate
        leftdata = YawRate * (r - ROBOT_RADIUS)
        rightdata = YawRate * (r + ROBOT_RADIUS)

    # 显示读取速度数据
    rospy.loginfo("Send vel:%f,%f", leftdata, rightdata)

    # 编辑发送的数据
    FH, FT = 0x88, 0X66
    data = chr(FH) + chr(FH)
    data = data + struct.pack("<f", leftdata)
    data = data + struct.pack("<f", rightdata)
    data = data + chr(getCrc8(data, 10))
    data = chr(FT) + chr(FT)

    # 发送数据
    ser.write(data)


def SerialRead(ser):
    """
    串口通信数据读取函数
    """
    global vel_left, vel_right, last_time_
    global x_, y_, th_, vx_, vy_, vth_
    global leftdata, rightdata
    data_FH, data_FT = 0x88, 0x66
    data_LEN = 13

    # 读取数据
    data = ser.read(data_LEN)

    # 接受到数据的情况下
    if len(data) == data_LEN:
        # 检查信息头
        if (ord(data[0]) != data_FH or ord(data[1]) != data_FH):
            rospy.logerr("Received message header error!")
            return 0
        
        # 检查信息尾
        if (ord(data[data_LEN - 2]) != data_FT
                or ord(data[data_LEN - 1]) != data_FT):
            rospy.logerr("Received message ender error!")
            return 0
        
        # 检查信息校验值
        if (ord(data[data_LEN - 3]) != getCrc8(data, data_LEN - 3)):
            rospy.logerr("Received data check sum error!")
            return 0
        
        # 获取数据
        vel_left = struct.unpack('<f', data[2:6])[0]
        vel_right = struct.unpack('<f', data[6:10])[0]

        # 显示读取速度数据
        rospy.loginfo("Read vel:%f,%f", vel_left, vel_right)

        # 积分计算里程计信息
        vx_ = (vel_right + vel_left) / 2
        vth_ = (vel_right - vel_left) / ROBOT_LENGTH

        # 运动参数运算
        dt = rospy.get_time() - last_time_
        delta_x = (vx_ * math.cos(th_) - vy_ * math.sin(th_)) * dt
        delta_y = (vx_ * math.sin(th_) + vy_ * math.cos(th_)) * dt
        delta_th = vth_ * dt
        x_ = x_ + delta_x
        y_ = y_ + delta_y
        th_ = th_ + delta_th
        last_time_ = rospy.get_time()


def DataUpdating():
    """
    数据更新函数
    """
    # 记录实时时间戳
    current_time_ = rospy.Time.now()

    # 定义广播器
    br = tf.TransformBroadcaster()

    # 发布TF
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th_)
    br.sendTransform((x_, y_, 0), odom_quat, current_time_, "base_footprint",
                     "odom")

    # 创建并发布里程计消息
    odometry = Odometry()
    odometry.header.stamp = current_time_
    odometry.header.frame_id = "odom"
    odometry.pose.pose.position.x = x_
    odometry.pose.pose.position.y = y_
    odometry.pose.pose.position.z = 0.0
    odometry.pose.pose.orientation.x = odom_quat[0]
    odometry.pose.pose.orientation.y = odom_quat[1]
    odometry.pose.pose.orientation.z = odom_quat[2]
    odometry.pose.pose.orientation.w = odom_quat[3]
    odometry.pose.covariance = odom_pose_covariance
    odometry.child_frame_id = "base_footprint"
    odometry.twist.twist.linear.x = vx_
    odometry.twist.twist.linear.y = vy_
    odometry.twist.twist.angular.z = vth_
    odometry.twist.covariance = odom_twist_covariance
    odom_pub.publish(odometry)


def callback(msg):
    """
    回调函数
    """
    global RobotV_, YawRate_
    RobotV_ = msg.linear.x
    YawRate_ = msg.angular.z


if __name__ == '__main__':
    try:
        # 初始化节点hhbot_bsp_node
        rospy.init_node('hhbot_bsp_node', anonymous=True)

        # 串口通信初始化
        bsp_ser = bsp_serial.SerialInit()

        # 定义发布器 odom_pub 发布 odom
        odom_pub = rospy.Publisher('odom', Odometry, queue_size=100)

        # 初始化订阅器 订阅cmd_vel
        rospy.Subscriber("cmd_vel", Twist, callback)

        # 初始化循环频率50HZ的对象rate
        rate = rospy.Rate(50)

        # 在程序没退出的情况下
        while not rospy.is_shutdown():
            # 串口通信数据发送
            SerialWrite(bsp_ser, RobotV_, YawRate_)

            # 串口通信数据读取
            SerialRead(bsp_ser)

            # 数据更新函数
            DataUpdating()

            # 休眠
            rate.sleep()

        # 关闭串口通信并显示测试结束
        bsp_ser.close()
    except rospy.ROSInterruptException:
        pass
