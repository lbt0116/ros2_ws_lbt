#ifndef _INTERFACE_H_
#define _INTERFACE_H_

#include <iostream>

#include "udp.h"

class Interface
{
public:
    Interface();
    ~Interface();

    void SetMotorData(LowCmd& cmd);
    void GetMotorData(LowState& state);

private:
    // 零位补偿
    void SetZeroPosOffset(LowState& state);
    void SetZeroPosOffset(LowCmd& cmd);

    StateList CombineState(StateList& state_0, StateList& state_1);

    UdpClient* module_0_;
    UdpClient* module_1_;

    // 补偿参数
    float pos_abad_left_;   // 肩关节
    float pos_hip_left_;    // 髋关节
    float pos_knee_left_;   // 膝关节
    float pos_abad_right_;  // 肩关节
    float pos_hip_right_;   // 髋关节
    float pos_knee_right_;  // 膝关节
    float pos_hip_left_4;   // 4leg髋关节

    // 单腿测试
    bool module_1_off_;
};

#endif  // _INTERFACE_H_