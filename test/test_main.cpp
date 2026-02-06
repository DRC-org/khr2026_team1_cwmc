
#ifdef NATIVE_TEST_ENV
#include <unity.h>
#include <math.h>
#include <stdint.h>
#include <cstring>

// Mock constants/types from main.cpp
#define PI 3.1415926535f
#define WHEEL_RADIUS 0.04925f
#define LX 0.1725f
#define LY 0.2425f
#define GEAR_RATIO 19.2032f
#define KP 1.0f
#define KI 0.05f
#define KD 50.0f
#define INTEGRAL_LIMIT 1000.0f
#define CLAMPING_OUTPUT 5000

// ==== KINEMATICS TESTS ====
void cmd_vel_to_rpm_test_logic(float vx, float vy, float vth, int16_t out_rpms[4]) {
  float v_fl = vx - vy - (LX + LY) * vth;
  float v_fr = vx + vy + (LX + LY) * vth;
  float v_rl = vx + vy - (LX + LY) * vth;
  float v_rr = vx - vy + (LX + LY) * vth;

  float rad_to_rpm_coeff = (60.0f * GEAR_RATIO) / (2.0f * PI * WHEEL_RADIUS);

  out_rpms[0] = static_cast<int16_t>(v_fl * rad_to_rpm_coeff);
  out_rpms[1] = static_cast<int16_t>(v_fr * rad_to_rpm_coeff);
  out_rpms[2] = static_cast<int16_t>(v_rl * rad_to_rpm_coeff);
  out_rpms[3] = static_cast<int16_t>(v_rr * rad_to_rpm_coeff);
}

void test_forward_kinematics_pure_x() {
    int16_t rpms[4];
    cmd_vel_to_rpm_test_logic(1.0f, 0.0f, 0.0f, rpms);
    
    TEST_ASSERT_INT16_WITHIN(10, rpms[0], rpms[1]); 
    TEST_ASSERT_INT16_WITHIN(10, rpms[2], rpms[3]);
    TEST_ASSERT_GREATER_THAN_INT16(0, rpms[0]);
}

void test_kinematics_rotation() {
    int16_t rpms[4];
    cmd_vel_to_rpm_test_logic(0.0f, 0.0f, 1.0f, rpms);

    TEST_ASSERT_LESS_THAN_INT16(0, rpms[0]); // FL
    TEST_ASSERT_GREATER_THAN_INT16(0, rpms[1]); // FR
    TEST_ASSERT_LESS_THAN_INT16(0, rpms[2]); // RL
    TEST_ASSERT_GREATER_THAN_INT16(0, rpms[3]); // RR
}

// ==== CURRENT CONVERSION TESTS ====
void milli_amperes_to_bytes(const int32_t milli_amperes[4], uint8_t out_tx_buf[8]) {
    for (uint8_t i = 0; i < 4; i++) {
        int32_t milli_ampere = milli_amperes[i] * 16384 / 20000;
        uint8_t upper = (milli_ampere >> 8) & 0xFF;
        uint8_t lower = milli_ampere & 0xFF;
        out_tx_buf[i * 2] = upper;
        out_tx_buf[i * 2 + 1] = lower;
    }
}

void test_zero_current() {
    int32_t inputs[4] = {0, 0, 0, 0};
    uint8_t output[8];
    memset(output, 0xFF, 8);
    
    milli_amperes_to_bytes(inputs, output);
    
    for (int i = 0; i < 8; i++) {
        TEST_ASSERT_EQUAL_UINT8(0, output[i]);
    }
}

void test_max_current() {
    int32_t inputs[4] = {20000, 20000, 20000, 20000};
    uint8_t output[8];
    
    milli_amperes_to_bytes(inputs, output);
    
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_EQUAL_UINT8(0x40, output[i * 2]);
        TEST_ASSERT_EQUAL_UINT8(0x00, output[i * 2 + 1]);
    }
}

void test_negative_current() {
    int32_t inputs[4] = {-10000, -10000, -10000, -10000};
    uint8_t output[8];
    
    milli_amperes_to_bytes(inputs, output);
    
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_EQUAL_UINT8(0xE0, output[i * 2]);
        TEST_ASSERT_EQUAL_UINT8(0x00, output[i * 2 + 1]);
    }
}

void test_mixed_values() {
    int32_t inputs[4] = {10000, -5000, 0, 15000};
    uint8_t output[8];
    
    milli_amperes_to_bytes(inputs, output);
    
    TEST_ASSERT_EQUAL_UINT8(0x20, output[0]);
    TEST_ASSERT_EQUAL_UINT8(0x00, output[1]);
    
    TEST_ASSERT_EQUAL_UINT8(0xF0, output[2]);
    TEST_ASSERT_EQUAL_UINT8(0x00, output[3]);
    
    TEST_ASSERT_EQUAL_UINT8(0x00, output[4]);
    TEST_ASSERT_EQUAL_UINT8(0x00, output[5]);
    
    TEST_ASSERT_EQUAL_UINT8(0x30, output[6]);
    TEST_ASSERT_EQUAL_UINT8(0x00, output[7]);
}

// ==== FORWARD KINEMATICS TESTS ====
void rpm_to_velocity(const int16_t rpms[4], float* vx, float* vy, float* vth) {
    float rpm_to_vel_coeff = (2.0f * PI * WHEEL_RADIUS) / (60.0f * GEAR_RATIO);
    
    float v_fl = rpms[0] * rpm_to_vel_coeff;
    float v_fr = rpms[1] * rpm_to_vel_coeff;
    float v_rl = rpms[2] * rpm_to_vel_coeff;
    float v_rr = rpms[3] * rpm_to_vel_coeff;
    
    *vx = (v_fl + v_fr + v_rl + v_rr) / 4.0f;
    *vy = (-v_fl + v_fr + v_rl - v_rr) / 4.0f;
    *vth = (-v_fl + v_fr - v_rl + v_rr) / (4.0f * (LX + LY));
}

void test_forward_kinematics_pure_forward() {
    int16_t rpms[4] = {1000, 1000, 1000, 1000};
    float vx, vy, vth;
    
    rpm_to_velocity(rpms, &vx, &vy, &vth);
    
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, vy);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, vth);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.0f, vx);
}

void test_forward_kinematics_pure_lateral() {
    int16_t rpms[4] = {-1000, 1000, 1000, -1000};
    float vx, vy, vth;
    
    rpm_to_velocity(rpms, &vx, &vy, &vth);
    
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, vx);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, vth);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.0f, vy);
}

void test_forward_kinematics_pure_rotation() {
    int16_t rpms[4] = {-1000, 1000, -1000, 1000};
    float vx, vy, vth;
    
    rpm_to_velocity(rpms, &vx, &vy, &vth);
    
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, vx);
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, vy);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.0f, vth);
}

// ==== PID CONTROL TESTS ====
float compute_pid_output(float error, float prev_error, float* integral, bool target_is_zero) {
    if (target_is_zero) {
        *integral = 0.0f;
        return 0.0f;
    }
    
    // Update integral with anti-windup
    *integral += error;
    if (*integral > INTEGRAL_LIMIT) *integral = INTEGRAL_LIMIT;
    if (*integral < -INTEGRAL_LIMIT) *integral = -INTEGRAL_LIMIT;
    
    float d_error = error - prev_error;
    return KP * error + KI * (*integral) + KD * d_error;
}

void test_pid_zero_target() {
    float integral = 100.0f; // Should be reset
    float output = compute_pid_output(50.0f, 0.0f, &integral, true);
    
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, integral);
}

void test_pid_proportional_only() {
    float integral = 0.0f;
    float error = 100.0f;
    float output = compute_pid_output(error, 100.0f, &integral, false); // Same prev_error = no derivative
    
    // P=100, I=5 (100*0.05), D=0
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 105.0f, output);
}

void test_pid_integral_windup_limit() {
    float integral = 0.0f;
    
    // Accumulate large error
    for (int i = 0; i < 100; i++) {
        compute_pid_output(1000.0f, 1000.0f, &integral, false);
    }
    
    TEST_ASSERT_FLOAT_WITHIN(0.01f, INTEGRAL_LIMIT, integral);
}

// ==== CLAMPING TESTS ====
int32_t apply_clamping(int32_t value) {
    if (value > CLAMPING_OUTPUT) return CLAMPING_OUTPUT;
    if (value < -CLAMPING_OUTPUT) return -CLAMPING_OUTPUT;
    return value;
}

void test_clamping_positive() {
    TEST_ASSERT_EQUAL_INT32(CLAMPING_OUTPUT, apply_clamping(10000));
}

void test_clamping_negative() {
    TEST_ASSERT_EQUAL_INT32(-CLAMPING_OUTPUT, apply_clamping(-10000));
}

void test_clamping_within_range() {
    TEST_ASSERT_EQUAL_INT32(1000, apply_clamping(1000));
    TEST_ASSERT_EQUAL_INT32(-1000, apply_clamping(-1000));
}

// ==== ANGLE NORMALIZATION TESTS ====
float normalize_angle(float delta_theta) {
    if (delta_theta > PI) delta_theta -= 2.0f * PI;
    if (delta_theta < -PI) delta_theta += 2.0f * PI;
    return delta_theta;
}

void test_angle_wrap_positive() {
    float angle = 4.0f; // > PI
    float normalized = normalize_angle(angle);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, angle - 2*PI, normalized);
}

void test_angle_wrap_negative() {
    float angle = -4.0f; // < -PI
    float normalized = normalize_angle(angle);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, angle + 2*PI, normalized);
}

void test_angle_no_wrap() {
    float angle = 1.5f; // Within [-PI, PI]
    float normalized = normalize_angle(angle);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, angle, normalized);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    
    // Kinematics tests (2)
    RUN_TEST(test_forward_kinematics_pure_x);
    RUN_TEST(test_kinematics_rotation);
    
    // Current conversion tests (4)
    RUN_TEST(test_zero_current);
    RUN_TEST(test_max_current);
    RUN_TEST(test_negative_current);
    RUN_TEST(test_mixed_values);
    
    // Forward kinematics tests (3)
    RUN_TEST(test_forward_kinematics_pure_forward);
    RUN_TEST(test_forward_kinematics_pure_lateral);
    RUN_TEST(test_forward_kinematics_pure_rotation);
    
    // PID control tests (3)
    RUN_TEST(test_pid_zero_target);
    RUN_TEST(test_pid_proportional_only);
    RUN_TEST(test_pid_integral_windup_limit);
    
    // Clamping tests (3)
    RUN_TEST(test_clamping_positive);
    RUN_TEST(test_clamping_negative);
    RUN_TEST(test_clamping_within_range);
    
    // Angle normalization tests (3)
    RUN_TEST(test_angle_wrap_positive);
    RUN_TEST(test_angle_wrap_negative);
    RUN_TEST(test_angle_no_wrap);
    
    UNITY_END();
    return 0;
}
#else
int main() { return 0; }
#endif
