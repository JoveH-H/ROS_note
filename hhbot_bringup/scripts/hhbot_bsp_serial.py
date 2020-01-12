#!/usr/bin/env python
# coding=utf-8

import serial  # 导入serial包 串口通信


def SerialInit():
    """
    HHbot BSP 串口通信初始化配置函数

    默认配置端口hhbot_bsp（已绑定），波特率115200，连接超时0.01秒
    """
    try:
        # 配置USB设备端口
        ser = serial.Serial("/dev/hhbot_bsp", 115200, timeout=0.01)
        print("BSP serial init ok")
        return ser
    except:
        print("BSP serial init error")
        return None


def SerialTest():
    """
    HHbot BSP 串口通信测试函数

    测试前需要配置HHbot下层执行器（HHbot BSP）为通信回环模式，即接受以‘\r\n’结尾的数据并返回发送

    配置串口并发送'Hello HHbot BSP\r\n'数据并读取 HHbot BSP 发送的数据，读取完成后关闭串口通信完成测试
    """
    # 串口通信初始化配置函数
    ser = SerialInit()

    # 串口开启的情况下
    if ser.isOpen():

        # 显示开始测试
        print("Start BSP serial test")

        # 编辑并显示发送的数据
        send_data = b"Hello HHbot BSP\r\n"
        print("Send data: %r" % send_data)

        # 发送数据
        ser.write(send_data)

        # 读取并显示数据
        read_data = ser.read(len(send_data))
        print("Read data: %s" % read_data)

        # 关闭串口通信并显示测试结束
        ser.close()
        print("End of BSP serial test")


if __name__ == '__main__':
    try:
        SerialTest()
    except:
        pass
