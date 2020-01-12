#!/usr/bin/env python
# coding=utf-8

import rospy  # 导入rospy包 ROS的python客户端
from std_msgs.msg import String  # 导入std_msgs.msg包的String消息类型


# 回调函数
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)  # 消息打印到屏幕上


def listener():
    rospy.init_node('listener', anonymous=True)  # 初始化节点listener
    rospy.Subscriber("chatter", String, callback)  # 初始化订阅器 订阅chatter
    rospy.spin()  # 保持主进程一直循环


if __name__ == '__main__':
    listener()
