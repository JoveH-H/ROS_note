#include <ros/ros.h>
#include <serial/serial.h> // ROS内置串口功能包

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "hhbot_bsp_serial_test");

    // 创建节点句柄
    ros::NodeHandle n;

    // 设置循环的频率
    ros::Rate loop_rate(1);

    // 声明串口对象
    serial::Serial ser;

    // 默认配置端口hhbot_bsp（已绑定），波特率115200，连接超时0.01秒
    ser.setPort("/dev/hhbot_bsp");
    ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(10);
    ser.setTimeout(to);

    // 打开串口
    ser.open();

    //检测串口是否已经打开，并给出提示信息
    if (ser.isOpen())
    {
        ROS_INFO_STREAM("BSP serial init ok");
        ROS_INFO_STREAM("Start BSP serial test");
    }
    else
    {
        ROS_ERROR("BSP serial init error");
        ROS_ERROR("End of BSP serial test");
        return -1;
    }
    
    while (ros::ok())
    {
        // 显示并发送串口数据
        std::string send_data;
        send_data = "Hello HHbot BSP\r\n";
        ROS_INFO("Send data: Hello HHbot BSP");
        ser.write(send_data);

        // 接受并显示串口数据
        std::string read_data;
        ser.read(read_data, ser.available());
        ROS_INFO("Read data: %s", read_data.c_str());

        // 按照循环频率延时
        loop_rate.sleep();
    }

    // 关闭串口
    ser.close();

    return 0;
}