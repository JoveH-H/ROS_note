#!/usr/bin/env python    
# -*- coding: utf-8 -*-

import rospy  
import tf  
 
if __name__ == '__main__':  

    # 初始化节点名称 
    rospy.init_node('fixed_tf_broadcaster')
  
    # 定义广播器
    br = tf.TransformBroadcaster() 

    # 初始化循环频率10HZ的对象rate
    rate = rospy.Rate(10.0)  

    while not rospy.is_shutdown():  
        # 建立一个新的参考系，父参考系为turtle1，并且距离父参考系2米
        br.sendTransform((0.0, 2.0, 0.0),  
                         (0.0, 0.0, 0.0, 1.0),  
                         rospy.Time.now(),  
                         "carrot1",  
                         "turtle1")   
        rate.sleep()	# 休眠
