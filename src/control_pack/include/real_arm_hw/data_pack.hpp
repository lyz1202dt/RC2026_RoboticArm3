#pragma once

namespace real_arm_hardware {

#pragma pack(push, 1)

struct Motor_t {
    float rad;
    float omega;
    float torque;
};

struct Arm_t {
    int pack_type;
    Motor_t joints[6];
    int air_pump_enable;
};

#pragma pack(pop)

}  // namespace real_arm_hardware
