#include "robot_hardware/imu/serial.h"

Serial::Serial()
{
    serial_descriptor_ = -1;
}

Serial::~Serial()
{
}

void Serial::OpenSerial(std::string dev_path,
                        BaudRate rate,
                        DataSize data_size,
                        CheckType check_type,
                        StopSize stop_size)
{
    // 开启端口 O_NOCTTY不作为这个端口的控制终端，O_NDELAY不关心端口另一端是否激活或者停止
    serial_descriptor_ = open(dev_path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_descriptor_ < 0)
    {
        std::cerr << "Can not open serial port : " << dev_path << std::endl;
        exit(-1);
    }
    std::cout << "open imu serial port." << std::endl;

    // 设置描述符为默认配置，即阻塞读取
    int file_control_ret = fcntl(serial_descriptor_, F_SETFL, 0);
    if (file_control_ret < 0)
    {
        std::cerr << "imu fd file set to default faile!" << std::endl;
        exit(-1);
    }

    // 确认设备是否是终端设备(判断是否工作正常，是否被占用)
    if (isatty(STDIN_FILENO) == 0)
    {
        std::cerr << "standard input is not a terminal device, imu \
            device was not work well."
                  << std::endl;
        exit(-1);
    }

    // 配置
    int result = tcgetattr(serial_descriptor_, &serial_option_);  // 保存旧的配置
    if (result == -1)
    {
        std::cerr << "can not get standard original serial port settings!" << std::endl;
        exit(-1);
    }
    struct termios option_new;
    memset(&option_new, 0, sizeof(option_new));
    // 默认配置
    option_new.c_cflag |= CLOCAL;  // 确保程序在突发的作业控制或挂起时，不会成为端口的占有者
    option_new.c_cflag |= CREAD;   // 启动接收

    // 设置输入的波特率
    switch (rate)
    {
        case BaudRate::RATE_2400: cfsetspeed(&option_new, B2400); break;
        case BaudRate::RATE_4800: cfsetspeed(&option_new, B4800); break;
        case BaudRate::RATE_9600: cfsetspeed(&option_new, B9600); break;
        case BaudRate::RATE_19200: cfsetspeed(&option_new, B19200); break;
        case BaudRate::RATE_38400: cfsetspeed(&option_new, B38400); break;
        case BaudRate::RATE_57600: cfsetspeed(&option_new, B57600); break;
        case BaudRate::RATE_115200: cfsetspeed(&option_new, B115200); break;
        case BaudRate::RATE_460800: cfsetspeed(&option_new, B460800); break;
        case BaudRate::RATE_921600: cfsetspeed(&option_new, B921600); break;

        default: break;
    }

    // 设置数据位
    option_new.c_cflag &= (~CSIZE);
    switch (data_size)
    {
        case DataSize::SIZE_5bit: option_new.c_cflag |= CS5; break;
        case DataSize::SIZE_6bit: option_new.c_cflag |= CS6; break;
        case DataSize::SIZE_7bit: option_new.c_cflag |= CS7; break;
        case DataSize::SIZE_8bit: option_new.c_cflag |= CS8; break;

        default: break;
    }

    // 设置校验位
    switch (check_type)
    {
        case CheckType::NONE: option_new.c_cflag &= (~PARENB); break;
        case CheckType::ODD:  // 奇校验
            option_new.c_cflag |= PARENB;
            option_new.c_cflag |= PARODD;
            option_new.c_iflag |= (INPCK | ISTRIP);
            break;
        case CheckType::EVEN:  // 偶校验
            option_new.c_cflag |= PARENB;
            option_new.c_cflag &= (~PARODD);
            option_new.c_iflag |= (INPCK | ISTRIP);
            break;

        default: break;
    }

    // 设置停止位
    switch (stop_size)
    {
        case StopSize::STOP_1bit: option_new.c_cflag &= (~CSTOPB); break;
        case StopSize::STOP_2bit: option_new.c_cflag |= CSTOPB; break;

        default: break;
    }

    // 设置最少字符和等待时间
    option_new.c_cc[VMIN] = 0;
    option_new.c_cc[VTIME] = 0;

    // 输入输出设置
    option_new.c_iflag |= IGNPAR;                          // 忽略桢错误和奇偶校验错
    option_new.c_oflag &= (~OPOST);                        // 不对输出数据进行处理
    option_new.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); /*raw input*/

    // 刷新输入输出队列
    tcflush(serial_descriptor_, TCIOFLUSH);
    // 设置生效
    result = tcsetattr(serial_descriptor_, TCSANOW, &option_new);
    if (result == -1)
    {
        std::cerr << "failed to set serial config!" << std::endl;
        exit(-1);
    }
}

void Serial::Close()
{
    // 恢复串口原始配置
    if (serial_descriptor_ > 0)
    {
        std::cout << "reset imu serial port options..." << std::endl;
        tcsetattr(serial_descriptor_, TCSADRAIN, &serial_option_);
    }
    close(serial_descriptor_);
}

int Serial::Send(char* buffer, int length)
{
    int result = write(serial_descriptor_, buffer, length);
    return result;
}

int Serial::Receive(char* buffer, int length)
{
    FD_ZERO(&read_descriptor_);
    FD_SET(serial_descriptor_, &read_descriptor_);
    time_out_.tv_sec = 1;
    time_out_.tv_usec = 0;
    int ready = select(serial_descriptor_ + 1, &read_descriptor_, NULL, NULL, &time_out_);
    int result = 0;

    if (ready > 0)
    {
        if (FD_ISSET(serial_descriptor_, &read_descriptor_)) result = read(serial_descriptor_, buffer, length);
    }
    else if (ready == 0)
    {
        std::cerr << "imu serial port reading time out, over 1 seconds!" << std::endl;
    }
    else
    {
        std::cerr << "imu serial port read error!" << std::endl;
    }

    return result;
}