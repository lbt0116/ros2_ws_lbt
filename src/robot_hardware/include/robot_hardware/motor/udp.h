#ifndef _UDP_H_
#define _UDP_H_

#include <string>
#include <sys/select.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <fcntl.h>
#include <thread>
#include <vector>
#include <mutex>
#include <chrono>

#include "data_type.h"
#include "tools.h"

class UdpClient
{
public:
    /**
     * @brief udp通信的构造函数
     * @param ip 电机通信服务的ip
     * @param port 电机通信服务的端口
     * @param module stm32模块的编号
     */
    UdpClient(std::string ip, int port, int stm32_index);
    ~UdpClient();

    /**
     * @brief 开启线程
     */
    void Start();

    /**
     * @brief 关闭线程
     */
    void Stop();

    /**
     * @brief 设置电机和指令
     * @param motor_data 6个电机的控制数据
     */
    void SetMotorDataAndOrder(std::vector<MotorCmd>& motor_data, InstructionList& order, int mode);

    /**
     * @brief 获取电机和机器人状态
     * @param motor_data 返回的6个电机的状态数据
     */
    void GetMotorAndRobotData(std::vector<MotorState>& motor_data, StateList& state, 
        float& inside_temperature, std::vector<float>& feet_force, int& mode);

private:

    /**
     * @brief 主循环
     */
    void Run();

    /**
     * @brief udp收发一次
     */
    void SendRecv();

    /**
     * @brief 字符拼接
     */
    unsigned short ByteToShort(unsigned char c1, unsigned char c2);

    /**
     * @brief 解析底板传回的电机数据
     */
    void ParseMotorData();

    // 通信相关
    int stm32_index_;
    std::string ip_;
    int port_;
    int socket_client_;
    struct sockaddr_in addr_server_;
    fd_set read_descriptor_;
    struct timeval time_out_;

    // 线程相关
    bool stop_;
    std::thread udp_thread_;
    std::mutex send_mutex_;
    std::mutex recv_mutex_;

    // 数据相关
    unsigned char* send_buffer_;
    unsigned char* recv_buffer_;
    std::vector<MotorData> output_;
    RobotData robot_data_;
    RobotErrorCode robot_error_code_;
    MotorErrorCode motor_error_code_;
};

#endif // _UDP_H_