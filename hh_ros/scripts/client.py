#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
from hh_ros.srv import *


def summation_client(x, y):
    # 等待接入服务节点summation
    rospy.wait_for_service('summation')
    try:
        # 定义服务的处理句柄
        summation_client = rospy.ServiceProxy('summation', Summation)
        print("Requesting %s+%s" % (x, y))
        resp1 = summation_client(x, y)
        return resp1.Sum
    except rospy.ServiceException, e:
        print("Service call failed: %s" % e)


def usage():
    return "%s [x y]" % sys.argv[0]


if __name__ == "__main__":
    # 从终端命令行获取两个加数
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    print("%s + %s = %s" % (x, y, summation_client(x, y)))

