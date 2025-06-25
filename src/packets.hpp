#ifndef PACKETS_HPP
#define PACKETS_HPP

#include <cstdint>

union vec3 {
    struct {
        float z;
        float y;
        float x;
    };
    struct {
        float yaw;
        float pitch;
        float roll;
    };
    float d[3];
};

struct StatePacket {
    uint32_t timestamp;
    vec3 orientation;
    float orientation_target;
    float P;
    float I;
    float D;
    float velocity;
    float velocity_target;
    float vP;
    float vI;
    float vD;
    float yaw_target;
    float yP;
    float yI;
    float yD;
};

struct PIDConfigurationPacket {
    float Kp;
    float Ki;
    float Kd;
};

struct ControlPacket {
    float x;
    float y;
};

enum class MotionPacketType : uint32_t {
    INVALID = 0,
    STATE = 1,
    CONTROL = 2,
    PID_CONFIGURATION_TILT = 3,
    PID_CONFIGURATION_VELOCITY = 4,
    PID_CONFIGURATION_YAW = 5,
};

struct MotionPacket {
    MotionPacketType type;
    union {
        StatePacket state;
        PIDConfigurationPacket pid_configuration;
        ControlPacket control;
    };
};

#endif /* PACKETS_HPP */