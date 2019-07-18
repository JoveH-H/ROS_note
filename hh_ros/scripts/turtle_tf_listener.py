#!/usr/bin/env python    
# -*- coding: utf-8 -*-

import rospy  
import math  
import tf  
import turtlesim.srv  
import geometry_msgs.msg
 
if __name__ == '__main__':

    # 初始化节点名称  
    rospy.init_node('turtle_listener')  

    # TransformListener创建后就开始接受tf广播信息，最多可以缓存10s  
    listener = tf.TransformListener() 

    # 使用spawn命令创建乌龟
    rospy.wait_for_service('spawn')  
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)  

    # turtle1已经默认生成，turtle2在地图上的初始坐标（4, 2, 0）生成
    spawner(4, 2, 0, 'turtle2')   

    # 定义话题发布节点，turtle2速度话题
    turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)  
 
    rate = rospy.Rate(10.0)  # 初始化循环频率10HZ的对象rate

    while not rospy.is_shutdown():  
        try:  
            # 获取源坐标系turtle2与目标坐标系之间最新的一次坐标转换
            (trans,rot) = listener.lookupTransform('/turtle2', '/carrot1', rospy.Time(0))  
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):  
            continue  

        # 发布turtle2速度，控制turtle2向turtle1靠近
        angular = 4 * math.atan2(trans[1], trans[0])  
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)  
	cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        turtle_vel.publish(cmd) 
 
        rate.sleep()	# 休眠

