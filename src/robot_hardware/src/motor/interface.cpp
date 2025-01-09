#include "robot_hardware/motor/interface.h"

#include <utility>
Interface::Interface()
    : module_1_off_(true)
{
    module_0_ = new UdpClient("192.168.1.100", 8000, 0);
    module_0_->Start();

    if (module_1_off_)
    {
        module_1_ = new UdpClient("192.168.2.100", 8000, 0);
        module_1_->Start();
    }

    pos_abad_left_ = -0.524;
    pos_hip_left_ = -2.4427;
    pos_knee_left_ = 0.65;
    pos_abad_right_ = -0.524;
    pos_hip_right_ = -1.05;
    pos_knee_right_ = -2.74;
    pos_hip_left_4 = -6.63;
}

Interface::~Interface()
{
    module_0_->Stop();
    delete module_0_;

    if (module_1_off_)
    {
        module_1_->Stop();
        delete module_1_;
    }
}

void Interface::SetMotorData(LowCmd& cmd)
{
    // 变换

    // 零位补偿
    SetZeroPosOffset(cmd);

    // 前6个电机
    std::vector<MotorCmd> motor_data_0;
    motor_data_0.resize(6);

    for (size_t i = 0; i < 6; i++)
    {
        motor_data_0[i] = cmd.motor_cmd[i];
    }

    // 后6个电机
    std::vector<MotorCmd> motor_data_1;
    motor_data_1.resize(6);

    for (size_t i = 0; i < 6; i++)
    {
        motor_data_1[i] = cmd.motor_cmd[i + 6];
    }

    // motor_data_1[0] = cmd.motor_cmd[3];

    module_0_->SetMotorDataAndOrder(motor_data_0, cmd.order, cmd.mode);

    if (module_1_off_) module_1_->SetMotorDataAndOrder(motor_data_1, cmd.order, cmd.mode);
}

void Interface::GetMotorData(LowState& state)
{
    std::vector<MotorState> motor_data_0, motor_data_1;
    motor_data_0.resize(6);
    motor_data_1.resize(6);
    StateList state_0, state_1;
    float inside_temperature_0, inside_temperature_1;
    std::vector<float> feet_force_0, feet_force_1;
    feet_force_0.resize(2);
    feet_force_1.resize(2);
    int mode_0, mode_1;

    module_0_->GetMotorAndRobotData(motor_data_0, state_0, inside_temperature_0, feet_force_0, mode_0);

    if (module_1_off_)
    {
        module_1_->GetMotorAndRobotData(motor_data_1, state_1, inside_temperature_1, feet_force_1, mode_1);
    }

    // 赋值
    state.mode = mode_0;
    state.state_list = CombineState(state_0, state_1);
    // 模块0是电源模块的温度，模块1是机舱内部温度
    state.inside_temperature = inside_temperature_1;
    state.feet_force[0] = feet_force_0[0];
    state.feet_force[1] = feet_force_0[1];
    state.feet_force[2] = feet_force_1[0];
    state.feet_force[3] = feet_force_1[1];

    for (size_t i = 0; i < 6; i++)
    {
        state.motor_state[i] = motor_data_0[i];
    }

    for (size_t i = 0; i < 6; i++)
    {
        state.motor_state[i + 6] = motor_data_1[i];
    }

    // 零位补偿
    SetZeroPosOffset(state);

    // 变换
    auto temp_state = state.motor_state;
    std::swap(state.motor_state[0], state.motor_state[6]);
    std::swap(state.motor_state[1], state.motor_state[7]);
    std::swap(state.motor_state[2], state.motor_state[8]);

    state.motor_state[0].position = -state.motor_state[0].position;
    state.motor_state[0].torque = -state.motor_state[0].torque;
    state.motor_state[0].velocity = -state.motor_state[0].velocity;

    state.motor_state[1].position = -state.motor_state[1].position;
    state.motor_state[1].torque = -state.motor_state[1].torque;
    state.motor_state[1].velocity = -state.motor_state[1].velocity;

    state.motor_state[2].position = -state.motor_state[2].position;
    state.motor_state[2].torque = -state.motor_state[2].torque;
    state.motor_state[2].velocity = -state.motor_state[2].velocity;

    state.motor_state[4].position = -state.motor_state[4].position;
    state.motor_state[4].torque = -state.motor_state[4].torque;
    state.motor_state[4].velocity = -state.motor_state[4].velocity;

    state.motor_state[5].position = -state.motor_state[5].position;
    state.motor_state[5].torque = -state.motor_state[5].torque;
    state.motor_state[5].velocity = -state.motor_state[5].velocity;

    state.motor_state[6].position = -state.motor_state[6].position;
    state.motor_state[6].torque = -state.motor_state[6].torque;
    state.motor_state[6].velocity = -state.motor_state[6].velocity;

    // state.motor_state[0] = temp_state[6];
    // state.motor_state[1] = temp_state[7];
    // state.motor_state[2] = temp_state[8];
    // state.motor_state[3] = temp_state[3];
    // state.motor_state[4] = temp_state[4];
    // state.motor_state[5] = temp_state[5];

    // state.motor_state[6] = temp_state[0];
    // state.motor_state[7] = temp_state[1];
    // state.motor_state[8] = temp_state[2];
    // state.motor_state[9] = temp_state[9];
    // state.motor_state[10] = temp_state[10];
    // state.motor_state[11] = temp_state[11];
}

StateList Interface::CombineState(StateList& state_0, StateList& state_1)
{
    StateList result = state_0;

    for (size_t i = 0; i < 6; i++)
    {
        result.motor_connect[i + 6] = state_1.motor_connect[i];
    }

    result.id_error = state_0.id_error || state_1.id_error;

    result.all_motor_poweron = state_0.all_motor_poweron && state_1.all_motor_poweron;

    return result;
}

void Interface::SetZeroPosOffset(LowState& state)
{
    state.motor_state[0].position += pos_abad_left_;
    state.motor_state[1].position += pos_hip_left_;
    state.motor_state[2].position += pos_knee_left_;
    state.motor_state[3].position += pos_abad_right_;
    state.motor_state[4].position += pos_hip_right_;
    state.motor_state[5].position += pos_knee_right_;
    state.motor_state[6].position += pos_abad_right_;
    state.motor_state[7].position += pos_hip_right_;
    state.motor_state[8].position += pos_knee_right_;
    state.motor_state[9].position += pos_abad_left_;
    state.motor_state[10].position += pos_hip_left_4;
    state.motor_state[11].position += pos_knee_left_;
}

void Interface::SetZeroPosOffset(LowCmd& cmd)
{
    cmd.motor_cmd[0].position -= pos_abad_left_;
    cmd.motor_cmd[1].position -= pos_hip_left_;
    cmd.motor_cmd[2].position -= pos_knee_left_;
    cmd.motor_cmd[3].position -= pos_abad_right_;
    cmd.motor_cmd[4].position -= pos_hip_right_;
    cmd.motor_cmd[5].position -= pos_knee_right_;
    cmd.motor_cmd[6].position -= pos_abad_right_;
    cmd.motor_cmd[7].position -= pos_hip_right_;
    cmd.motor_cmd[8].position -= pos_knee_right_;
    cmd.motor_cmd[9].position -= pos_abad_left_;
    cmd.motor_cmd[10].position -= pos_hip_left_4;
    cmd.motor_cmd[11].position -= pos_knee_left_;
}