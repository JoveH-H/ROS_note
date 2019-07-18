#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf  
import turtlesim.msg  
 
 
def handle_turtle_pose(msg, turtlename):  
    
    # 定义广播器
    br = tf.TransformBroadcaster()  

    # 发布乌龟的平移和翻转，发布从world坐标系到turtleX坐标系的变换
    br.sendTransform((msg.x, msg.y, 0),  
                     tf.transformations.quaternion_from_euler(0, 0, msg.theta),  
                     rospy.Time.now(),  
                     turtlename,  
                     "world")    
 
 
if __name__ == '__main__': 
 
    # 初始化节点名称 
    rospy.init_node('turtle_tf_broadcaster')  

    # 获取海龟的名字（turtle1，turtle2）
    turtlename = rospy.get_param('~turtle')

    # 订阅海龟的pose话题，并发布
    rospy.Subscriber('/%s/pose' % turtlename,  
                     turtlesim.msg.Pose,  
                     handle_turtle_pose,  
                     turtlename)   

    rospy.spin() # 保持主进程一直循环

