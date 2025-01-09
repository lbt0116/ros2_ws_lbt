#include "robot_hardware/motor/udp.h"

UdpClient::UdpClient(std::string ip, int port, int stm32_index) :
    ip_(ip), port_(port), stm32_index_(stm32_index), stop_(false)
{
    // 创建udp套接字
    socket_client_ = socket(AF_INET, SOCK_DGRAM, 0);

    if (socket_client_ < 0)
    {
        std::cerr << "socket create failed!" << std::endl;
        exit(-1);
    }

    // 设置服务器地址
    int addr_length = sizeof(addr_server_);
    memset(&addr_server_, 0, addr_length);
    addr_server_.sin_family = AF_INET;
    addr_server_.sin_port = htons(port_);
    addr_server_.sin_addr.s_addr = inet_addr(ip_.c_str());

    // 数据
    send_buffer_ = new unsigned char[1024];
    recv_buffer_ = new unsigned char[1024];
    memset(send_buffer_, 0, BUFFER_SIZE);
    memset(recv_buffer_, 0, BUFFER_SIZE);
    memset(&robot_data_, 0, sizeof(robot_data_));
    memset(&robot_error_code_, 0, sizeof(robot_error_code_));
    memset(&motor_error_code_, 0, sizeof(motor_error_code_));

    output_.resize(6);
    for (size_t i = 0; i < 6; i++)
    {
        output_[i] = MotorData{0};
    }
}

UdpClient::~UdpClient()
{
    delete[] send_buffer_;
    delete[] recv_buffer_;
}

void UdpClient::Run()
{
    // using clock = std::chrono::high_resolution_clock;
    while (true)
    {
        // auto start = clock::now();
        // 退出
        if (stop_)
        {
            break;
        }

        SendRecv();

        // auto end = clock::now();
        // std::chrono::duration<double> time = end - start;
        // std::cout << "Elapsed time: " << time.count() << " s" << std::endl;
    }

    std::cerr << "Exit the udp thread..." << std::endl;
}

void UdpClient::SendRecv()
{
    // 发送数据
    int send_length;
    {
        std::unique_lock<std::mutex> lock(send_mutex_);
        send_length = sendto(socket_client_, send_buffer_, SEND_DATA_LENGTH, 0, 
            (struct sockaddr*)&addr_server_, sizeof(addr_server_));
    }

    if (send_length < 0)
    {
        perror("udp send data error!");
        exit(-1);
    }

    // 判断超时，超时时间为50ms
    FD_ZERO(&read_descriptor_);
    FD_SET(socket_client_, &read_descriptor_);
    time_out_.tv_sec = 0;
    time_out_.tv_usec = 50000;
    int ready = select(socket_client_ + 1, &read_descriptor_, NULL, NULL, &time_out_);
    if (ready == 0)
    {
        std::cerr << "udp socket reading time out!" << std::endl;
        return;
    }

    // 接收数据
    int recv_length;      
    recv_length = recvfrom(socket_client_, recv_buffer_, BUFFER_SIZE, 0,
        nullptr, nullptr);
    
    if (recv_length > 0)
    {
        ParseMotorData();
    }
    else if (recv_length == 0)
    {
        // 连接断开
        close(socket_client_);
        FD_CLR(socket_client_, &read_descriptor_);
        std::cerr << "Server socket has been closed!" << std::endl;
        exit(-1);
    }
    else
    {
        // 接收数据失败
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            // 暂无数据可读，可继续读
            std::cerr << "No data to read, read again! erron code = " << errno << std::endl;
            return;
        }

        if (errno == EINTR)
        {
            // 被中断，可继续读
            std::cerr << "Interrupted, read again! erron code = " << errno << std::endl;
            return;
        }

        // 异常退出
        std::cerr << "Receive data error, error code = " << errno << std::endl;
        exit(-1);
    }
}

void UdpClient::Stop()
{
    stop_ = true;
    udp_thread_.join();
}

void UdpClient::Start()
{
    // 开启线程
    udp_thread_ = std::thread(&UdpClient::Run, this);
}

void UdpClient::SetMotorDataAndOrder(std::vector<MotorCmd>& motor_data, InstructionList& order, int mode)
{
    // 判断数据大小
    if (motor_data.size() != 6)
    {
        std::cerr << "Send to motor data size not equal to 6, now size = " << motor_data.size() << std::endl;
        exit(-1);
    }

    std::unique_lock<std::mutex> lock(send_mutex_);
    Command cmd;
    cmd.value.spotlight = order.open_light;
    cmd.value.state_light = order.state_light;
    cmd.value.motor_enable = order.motor_enable;
    cmd.value.motor_disenable = order.motor_disenable;
    cmd.value.motor_power_supply = order.motor_power_supply;
    cmd.value.motor_charging_electrodes = order.motor_charging_electrodes;

    send_buffer_[0] = 0xFE;
    send_buffer_[1] = 0xEE;
    send_buffer_[2] = cmd.data;
    send_buffer_[3] = mode;

    for (size_t i = 0; i < motor_data.size(); i++)
    {
        send_buffer_[4 + i * 10] = (FloatToInt(motor_data[i].torque, T_MIN, T_MAX, 16) >> 8) & 0xFF;
        send_buffer_[5 + i * 10] = (FloatToInt(motor_data[i].torque, T_MIN, T_MAX, 16)) & 0xFF;
        send_buffer_[6 + i * 10] = (FloatToInt(motor_data[i].velocity, V_MIN, V_MAX, 16) >> 8) & 0xFF;
        send_buffer_[7 + i * 10] = (FloatToInt(motor_data[i].velocity, V_MIN, V_MAX, 16)) & 0xFF;
        send_buffer_[8 + i * 10] = (FloatToInt(motor_data[i].position, P_MIN, P_MAX, 16) >> 8) & 0xFF;
        send_buffer_[9 + i * 10] = (FloatToInt(motor_data[i].position, P_MIN, P_MAX, 16)) & 0xFF;
        send_buffer_[10 + i * 10] = (FloatToInt(motor_data[i].kp, KP_MIN, KP_MAX, 16) >> 8) & 0xFF;
        send_buffer_[11 + i * 10] = (FloatToInt(motor_data[i].kp, KP_MIN, KP_MAX, 16)) & 0xFF;
        send_buffer_[12 + i * 10] = (FloatToInt(motor_data[i].kd, KD_MIN, KD_MAX, 16) >> 8) & 0xFF;
        send_buffer_[13 + i * 10] = (FloatToInt(motor_data[i].kd, KD_MIN, KD_MAX, 16)) & 0xFF;
    }

    unsigned short check_sum = CrcVerify(send_buffer_, 4 + 10 * 6);

    send_buffer_[64] = (check_sum >> 8) & 0xFF;
    send_buffer_[65] = check_sum & 0xFF;
}

void UdpClient::GetMotorAndRobotData(std::vector<MotorState>& motor_data, StateList& state, 
    float& inside_temperature, std::vector<float>& feet_force, int& mode)
{
    // 判断数据大小
    if (motor_data.size() != 6)
    {
        std::cerr << "recv from motor data size not equal to 6, now size = " << motor_data.size() << std::endl;
        exit(-1);
    }
    if (feet_force.size() != 2)
    {
        std::cerr << "feet force data size not equal to 2, now size = " << feet_force.size() << std::endl;
        exit(-1);
    }

    std::unique_lock<std::mutex> lock(recv_mutex_);

    mode = robot_data_.mode;

    state.motor_connect[0] = robot_data_.error.value.motor_0;
    state.motor_connect[1] = robot_data_.error.value.motor_1;
    state.motor_connect[2] = robot_data_.error.value.motor_2;
    state.motor_connect[3] = robot_data_.error.value.motor_3;
    state.motor_connect[4] = robot_data_.error.value.motor_4;
    state.motor_connect[5] = robot_data_.error.value.motor_5;
    state.id_error = robot_data_.error.value.id_error;
    state.motor_time_out = robot_data_.error.value.motor_time_out;
    state.emergency_stop = robot_data_.error.value.emergency_stop;
    state.charging_state = robot_data_.error.value.charging_state;
    state.fan_0_error = robot_data_.error.value.fan_0_error;
    state.fan_1_error = robot_data_.error.value.fan_1_error;
    state.low_power = robot_data_.error.value.low_power;
    state.power_warning = robot_data_.error.value.power_warning;
    state.BMS_overtime = robot_data_.error.value.BMS_overtime;
    state.all_motor_poweron = robot_data_.error.value.all_motor_poweron;

    inside_temperature = robot_data_.temperature;

    feet_force[0] = robot_data_.feet_force[0];
    feet_force[1] = robot_data_.feet_force[1];
    
    for (size_t i = 0; i < 6; i++)
    {
        motor_data[i].torque = output_[i].torque;
        motor_data[i].velocity = output_[i].velocity;
        motor_data[i].position = output_[i].position;
        motor_data[i].temperature = output_[i].temperature;
        motor_data[i].run_state = output_[i].error.value.mode_state;
        // 故障
        motor_data[i].error.under_voltage = output_[i].error.value.under_voltage;
        motor_data[i].error.over_current = output_[i].error.value.over_current;
        motor_data[i].error.over_heat = output_[i].error.value.over_heat;
        motor_data[i].error.magnetic_encoder = output_[i].error.value.magnetic_encoder;
        motor_data[i].error.HALL_encoder = output_[i].error.value.HALL_encoder;
        motor_data[i].error.encoder_calibrated = output_[i].error.value.encoder_calibrated;
    }
}

void UdpClient::ParseMotorData()
{
    // 校验检查
    if (recv_buffer_[0] != 0xFE || recv_buffer_[1] != 0xEE)
        return;
    unsigned short check_sum = CrcVerify(recv_buffer_, 4 + 8 + 10 * 6);
    if (recv_buffer_[72] != (check_sum & 0xFF00) >> 8 || recv_buffer_[73] != (check_sum & 0x00FF))
        return;
    
    std::unique_lock<std::mutex> lock(recv_mutex_);
    // 机器人状态
    robot_data_.mode = recv_buffer_[3];
    robot_data_.error.data = ByteToShort(recv_buffer_[4], recv_buffer_[5]);
    robot_data_.temperature = (float)ByteToShort(recv_buffer_[6], recv_buffer_[7]) / 100.0;
    robot_data_.feet_force[0] = ByteToShort(recv_buffer_[8], recv_buffer_[9]);
    robot_data_.feet_force[1] = ByteToShort(recv_buffer_[10], recv_buffer_[11]);

    // 电机数据
    for (size_t i = 0; i < 6; i++)
    {
        float temp = ByteToShort(recv_buffer_[12 + 10 * i], recv_buffer_[13 + 10 * i]) / 10.0;
        unsigned char error = ByteToShort(recv_buffer_[14 + 10 * i], recv_buffer_[15 + 10 * i]);
        float tau = IntToFloat(ByteToShort(recv_buffer_[16 + 10 * i], recv_buffer_[17 + 10 * i]), T_MIN, T_MAX, 16);
        float velocity = IntToFloat(ByteToShort(recv_buffer_[18 + 10 * i], recv_buffer_[19 + 10 * i]), V_MIN, V_MAX, 16);
        float angle = IntToFloat(ByteToShort(recv_buffer_[20 + 10 * i], recv_buffer_[21 + 10 * i]), P_MIN, P_MAX, 16);

        output_[i].torque = tau;
        output_[i].velocity = velocity;
        output_[i].position = angle;
        output_[i].temperature = temp;
        output_[i].error.data = error;
    }
}

unsigned short UdpClient::ByteToShort(unsigned char c1, unsigned char c2)
{
    unsigned short data = 0;
    data = ((data | c1) << 8) | c2;
    return data;
}