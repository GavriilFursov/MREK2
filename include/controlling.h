#pragma once

inline float deg2rad(float angle) { return angle * DEG_TO_RAD; }
inline float rad2deg(float angle) { return angle * RAD_TO_DEG; }

const float L1 = 183.0f;
const float L2 = 80.0f;
const float BASE_ANGLE = 118.73f;
const float ZERO_LENGTH_HIP = 232.3002f; 

const long STEPS_PER_REV = 10000;
const float SCREW_PITCH = 5.0f;
const float MM_PER_STEP = SCREW_PITCH / STEPS_PER_REV;

inline float angleToLength(float target_angle) {
    float buf_rad = radians(BASE_ANGLE - target_angle);
    return sqrt(L1*L1 + L2*L2 - 2*L1*L2*cosf(buf_rad));
}

inline long angleToSteps(float target_angle) {
    float target_length = angleToLength(target_angle);
    float length_offset = ZERO_LENGTH_HIP - target_length;
    return length_offset / MM_PER_STEP;
}