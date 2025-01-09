#ifndef _SERIAL_H_
#define _SERIAL_H_

#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/signal.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <string>
#include <cstring>
#include <iostream>

// 数据缓冲区大小
const int BUFFER_LENGTH = 1024;

// 波特率
enum class BaudRate
{
    RATE_2400,
    RATE_4800,
    RATE_9600,
    RATE_19200,
    RATE_38400,
    RATE_57600,
    RATE_115200,
    RATE_460800,
    RATE_921600
};

// 数据位大小
enum class DataSize
{
    SIZE_8bit,
    SIZE_7bit,
    SIZE_6bit,
    SIZE_5bit
};

// 校验方式
enum class CheckType
{
    NONE, // 无校验
    ODD, // 奇校验
    EVEN // 偶校验
};

// 停止位
enum class StopSize
{
    STOP_1bit,
    STOP_2bit
};

class Serial
{
public:
    Serial();
    ~Serial();

    void OpenSerial(std::string dev_path, BaudRate rate, DataSize data_size,
        CheckType check_type, StopSize stop_size);
    // 关闭通信
    void Close();
    int Send(char* buffer, int length);
    int Receive(char* buffer, int length);

private:
    int serial_descriptor_; // 串口描述符
    struct termios serial_option_; // 串口原始配置

    fd_set read_descriptor_;
    struct timeval time_out_;
};

#endif // _SERIAL_H_