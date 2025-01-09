#ifndef _DATA_TYPE_H_
#define _DATA_TYPE_H_

#define SEND_DATA_LENGTH 66
#define BUFFER_SIZE 1024

// 电机相关的定义
#define P_MIN -12.5f 
#define P_MAX 12.5f 
#define V_MIN -15.0f 
#define V_MAX 15.0f 
#define KP_MIN 0.0f 
#define KP_MAX 5000.0f 
#define KD_MIN 0.0f 
#define KD_MAX 100.0f 
#define T_MIN -120.0f 
#define T_MAX 120.0f

union Command
{
    unsigned char data;

    struct Value
    {
        unsigned char state_light : 3; // 状态灯运行方式
        unsigned char spotlight : 1; // 打开照明灯，持续发送
        unsigned char motor_enable : 1; // 电机使能，发送一次即可
        unsigned char motor_disenable : 1; // 电机失能，发送一次即可
        unsigned char motor_power_supply : 1; // 需要一直发，且第一上电需要等待4s
        unsigned char motor_charging_electrodes : 1; // 自充电极，持续发送
    }value;
};

union RobotErrorCode
{
    unsigned short data;

    struct Value
    {
        unsigned char motor_0 : 1; // 通信故障
        unsigned char motor_1 : 1;
        unsigned char motor_2 : 1;
        unsigned char motor_3 : 1;
        unsigned char motor_4 : 1;
        unsigned char motor_5 : 1;
        unsigned char id_error : 1;
        unsigned char motor_time_out : 1;
        unsigned char emergency_stop : 1;
        unsigned char charging_state : 1;
        unsigned char fan_0_error : 1;
        unsigned char fan_1_error : 1;
        unsigned char low_power : 1;
        unsigned char power_warning : 1;
        unsigned char BMS_overtime : 1;
        unsigned char all_motor_poweron : 1; // 一组电机全部上电
    }value;
};

union MotorErrorCode
{
    unsigned short data;

    struct value
    {
        // 低八位为故障码
        unsigned char under_voltage : 1;
        unsigned char over_current : 1;
        unsigned char over_heat : 1;
        unsigned char magnetic_encoder : 1; // 磁编码故障
        unsigned char HALL_encoder : 1; // HALL编码故障
        unsigned char encoder_calibrated : 1; // 编码器未标定
        unsigned char mode_state : 2; // 0=复位 1=标定 2=运行
        // 高八位暂未使用
        unsigned char not_use : 8;
    }value;
};

// 电机数据
struct MotorData
{
    float torque;
    float velocity;
    float position;
    float kp;
    float kd;

    // 用于返回电机状态使用
    float temperature;
    MotorErrorCode error;
};

// 机器人数据
struct RobotData
{
    int mode;
    unsigned short feet_force[2];
    float temperature;
    RobotErrorCode error;
};

/************** 交互用 *****************/

// 指令表
struct InstructionList
{
    int state_light;
    int open_light;
    int motor_enable;
    int motor_disenable;
    int motor_power_supply;
    int motor_charging_electrodes;
};

// 电机指令
struct MotorCmd
{
    float torque;
    float velocity;
    float position;
    float kp;
    float kd;
};

// 上位指令
struct LowCmd
{
    InstructionList order;
    int mode;
    MotorCmd motor_cmd[12];
};

// 状态表
struct StateList
{
    int motor_connect[12];
    int id_error;
    int motor_time_out;
    int emergency_stop;
    int charging_state;
    int fan_0_error;
    int fan_1_error;
    int low_power;
    int power_warning;
    int BMS_overtime;
    int all_motor_poweron;
};

// 电机故障表
struct MotorErrorList
{
    int under_voltage;
    int over_current;
    int over_heat;
    int magnetic_encoder;
    int HALL_encoder;
    int encoder_calibrated;
};

// 电机状态
struct MotorState
{
    float torque;
    float velocity;
    float position;
    float temperature;
    int run_state;
    MotorErrorList error;
};

// 机器人返回状态
struct LowState
{
    int mode;
    StateList state_list;
    float inside_temperature;
    float feet_force[4];
    MotorState motor_state[12];
};

#endif // _DATA_TYPE_H_