#!/usr/bin/env python    
# -*- coding: utf-8 -*-

import rospy
import tf
import math

if __name__ == '__main__':

    # 初始化节点名称
    rospy.init_node('move_tf_broadcaster')

    # 定义广播器
    br = tf.TransformBroadcaster()

    # 初始化循环频率10HZ的对象rate
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():

        # 建立一个圆环变化的数
        t = rospy.Time.now().to_sec() * math.pi

        # 建立一个新的参考系，父参考系为turtle1，距离保持不变，位置在改变
        br.sendTransform((2.0 * math.sin(t), 2.0 * math.cos(t), 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "carrot1",
                         "turtle1")
        rate.sleep()	# 休眠
