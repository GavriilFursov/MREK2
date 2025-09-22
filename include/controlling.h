#pragma once

RS485Master leftHip(PA7, newSerial, 7);
RS485Master leftKnee(PA7, newSerial, 5);

struct limbSegment {
    float length;
    float zeroLength;
};

limbSegment hip = { 0.42f, 232.3002f };
limbSegment knee = { 0.44f, 238.06f };

float A0X = 0.0f,           A0Z = 0.0f;
float A1X = 0.0f,           A1Z = 0.0f;
float A2X = 0.0f,           A2Z = 0.0f;
float q1Des = 0.0f,         q21Des = 0.0f;
float xDes = 0.0f,          zDes = 0.0f;

const long STEPS_PER_REV = 10000;
const float SCREW_PITCH = 5.0f;
const float MM_PER_STEP = SCREW_PITCH / STEPS_PER_REV;

const float L1 = 183.0f;
const float L2 = 80.0f;
const float BASE_ANGLE = 118.73f;

const float ORIGINAL_FOURIER_CYCLE_TIME = 1.134f;
float time_scale_factor = 5.0f / 1.134f;

inline float angleToLength(float target_angle, const limbSegment& segment) {
    float buf_rad = radians(BASE_ANGLE - target_angle);
    return sqrt(L1*L1 + L2*L2 - 2*L1*L2*cosf(buf_rad));
}

inline long angleToSteps(float target_angle, const limbSegment& segment) {
    float target_length = angleToLength(target_angle, segment);
    float length_offset = segment.zeroLength - target_length;
    return (long)(length_offset / MM_PER_STEP);
}

inline float stepsToLength(long target_steps, const limbSegment& segment) {
    return segment.zeroLength - (target_steps * MM_PER_STEP);
}

inline float stepsToAngle(long target_steps, const limbSegment& segment) {
    float current_length = stepsToLength(target_steps, segment);
    float buf_rad = acosf((current_length*current_length - L1*L1 - L2*L2) / (-2.0f*L1*L2));
    return BASE_ANGLE - degrees(buf_rad);
}

inline void directKinematics(float q_hip, float q_knee) {
    A0X = 0.0f;
    A0Z = 0.0f;
    
    A1X = A0X + hip.length * sinf(q_hip);
    A1Z = A0Z - hip.length * cosf(q_hip);
    
    A2X = A1X + knee.length * sinf(q_hip + q_knee);
    A2Z = A1Z - knee.length * cosf(q_hip + q_knee);
}

inline void inverseKinematics(float x_set, float z_set, float rq1, float rq21, float x_start, float z_start) {
    q1Des = -(knee.length + z_set*cos(rq1 + rq21) - z_start*cos(rq1 + rq21) - x_set*sin(rq1 + rq21) + x_start*sin(rq1 + rq21) + hip.length*cos(rq21) - hip.length*rq1*sin(rq21)) / (hip.length*sin(rq21));
    q21Des = rq21 + ((knee.length*cos(rq1 + rq21) + hip.length*cos(rq1)) * (z_set - z_start + knee.length*cos(rq1 + rq21) + hip.length*cos(rq1))) / (hip.length*knee.length*sin(rq21)) + ((knee.length*sin(rq1 + rq21) + hip.length*sin(rq1)) * (x_start - x_set + knee.length*sin(rq1 + rq21) + hip.length*sin(rq1))) / (hip.length*knee.length*sin(rq21));
    
    q1Des = constrain(q1Des, -0.261799f, 1.48353f);
    q21Des = constrain(q21Des, -1.5708f, -0.0523599f);
}

inline void fourierTrajectory(float x) {

    static const struct {
        const float a0 = 0.275f;
        const float a1 = -0.212f, b1 = 0.08912f;
        const float a2 = -0.02665f, b2 = -0.04015f;
        const float a3 = 0.01219f, b3 = -4.207e-4f;
        const float w = 5.312f;
    } xp;
    
    static const struct {
        const float a0 = 0.09473f;
        const float a1 = 0.03905f, b1 = 0.001059f;
        const float a2 = -0.0002675f, b2 = 0.004628f;
        const float a3 = -0.01259f, b3 = 0.01509f;
        const float a4 = -0.007607f, b4 = 0.01752f;
        const float a5 = -0.001034f, b5 = 0.008936f;
        const float a6 = 0.002177f, b6 = 0.0009939f;
        const float a7 = 0.002133f, b7 = -0.0004353f;
        const float w = 4.395f;
    } zp;
    
    float wx = x * xp.w;
    xDes = xp.a0 + xp.a1*cosf(wx) + xp.b1*sinf(wx) + xp.a2*cosf(2*wx) + xp.b2*sinf(2*wx) + xp.a3*cosf(3*wx) + xp.b3*sinf(3*wx);
    
    float wz = x * zp.w;
    zDes = zp.a0 + zp.a1*cosf(wz) + zp.b1*sinf(wz) + zp.a2*cosf(2*wz) + zp.b2*sinf(2*wz) + zp.a3*cosf(3*wz) + zp.b3*sinf(3*wz) + 
           zp.a4*cosf(4*wz) + zp.b4*sinf(4*wz) + zp.a5*cosf(5*wz) + zp.b5*sinf(5*wz) + zp.a6*cosf(6*wz) + zp.b6*sinf(6*wz) + 
           zp.a7*cosf(7*wz) + zp.b7*sinf(7*wz);
}

inline void updateFourierTrajectory() {
    float current_time = fmod(millis() / 1000.0f, ORIGINAL_FOURIER_CYCLE_TIME * time_scale_factor);
    float normalized_time = current_time / (ORIGINAL_FOURIER_CYCLE_TIME * time_scale_factor);
    fourierTrajectory(normalized_time);
}

inline void setStopMotors() {
    // lefthip.run(0);
    // leftKnee.run(0);
}

inline void handleSerialCommand() {
    if(Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        if (input.equalsIgnoreCase("Stop")) {
            setStopMotors();
            system_state = SS_MAIN_MENU;
            return;
        } 
        else if (input.equalsIgnoreCase("Test")) {
            system_state = SS_TEST;
            return;
        }       
    }
}

void mainControl() {
    handleSerialCommand();
    switch (system_state) {
        case SS_MAIN_MENU: {
        
        } break;
        
        case SS_TEST: {
            static int8_t mode = 0;
            static float startA2X = 0.0f, startA2Z = 0.0f;
            static float target_q1 = 0.0f, target_q21 = 0.0f;
            const int INTERPOLATION_STEPS = 100;

            switch (mode) {
                case 0: {

                    float current_q1 = radians(stepsToAngle(leftHip.getPosition(), hip));
                    float current_q21 = -1 * radians(stepsToAngle(leftKnee.getPosition(), knee));
                    
                    directKinematics(current_q1, current_q21);
                    startA2X = A2X; 
                    startA2Z = A2Z;
                    
                    fourierTrajectory(0.0f);
                    xDes = xDes - 0.32; zDes = zDes = 0.88;
                                        
                    float step_x = (xDes - startA2X) / INTERPOLATION_STEPS;
                    float step_z = (zDes - startA2Z) / INTERPOLATION_STEPS;
                    
                    float interpolated_q1 = current_q1;
                    float interpolated_q21 = current_q21;
                    
                    for(int i = 0; i <= INTERPOLATION_STEPS; i++) {
                        float interp_x = startA2X + (i * step_x);
                        float interp_z = startA2Z + (i * step_z);
                        
                        inverseKinematics(interp_x, interp_z, interpolated_q1, interpolated_q21, A0X, A0Z);
                        
                        interpolated_q1 = q1Des;
                        interpolated_q21 = q21Des;
                    }
                    
                    target_q1 = interpolated_q1;
                    target_q21 = interpolated_q21;

                } break;
            
                case 1: {

                } break;
            }

        } break;
    }   
}   