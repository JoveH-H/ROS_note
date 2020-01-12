#!/usr/bin/env python
# coding=utf-8

import rospy  # 导入rospy包 ROS的python客户端
from std_msgs.msg import String  # 导入std_msgs.msg包的String消息类型


# 定义发布函数 talker
def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)  # 定义发布器 pub 发布chatter
    rospy.init_node('talker', anonymous=True)  # 初始化节点talker
    rate = rospy.Rate(10)  # 初始化循环频率10HZ的对象rate

    # 在程序没退出的情况下
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)  # 消息打印到屏幕上
        pub.publish(hello_str)  # 发布消息
        rate.sleep()  # 休眠


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
