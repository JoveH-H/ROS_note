#!/usr/bin/env python
# coding=utf-8

from hh_ros.srv import *  # 导入定义的服务
import rospy  # 导入rospy包 ROS的python客户端


# 回调函数
def handle_summation(req):
    # 显示输入参数中的请求数据相加过程和结果
    print("Returning [%s + %s = %s]" % (req.A, req.B, (req.A + req.B)))

    # SummationResponse由Summation服务生成的返回函数，返回相加结果
    return SummationResponse(req.A + req.B)


# 声明一个名为summation的新服务
def summation_server():
    # 声明节点为summation_server
    rospy.init_node('summation_server')

    # 定义服务节点名称，服务的类型，回调函数
    s = rospy.Service('summation', Summation, handle_summation)

    print("Ready to summation.")
    rospy.spin()


if __name__ == "__main__":
    try:
        summation_server()
    except rospy.ROSInterruptException:
        pass
