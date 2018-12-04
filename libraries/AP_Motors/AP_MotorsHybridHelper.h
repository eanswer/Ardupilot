#ifndef __AP_MOTORS_HELPER_H__
#define __AP_MOTORS_HELPER_H__

#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library

// Definitions of helper functions.
float max(const float a, const float b) {
    if (a > b) {
        return a;
    } else {
        return b;
    }
}

int max(const int a, const int b) {
    if (a > b) {
        return a;
    } else {
        return b;
    }
}

float remap(const float in_value, const float in_low, const float in_high,
        const float out_low, const float out_high) {
    if (fabs(in_low - in_high) < 1e-5) return (out_low + out_high) / 2.0f;
    return (in_value - in_low) / (in_high - in_low) * (out_high - out_low) + out_low;
}

float clamp(const float in_value, const float in_low, const float in_high) {
    return (in_value < in_low ? in_low : (in_value > in_high ? in_high : in_value));
}

float deg2rad(const float x) {
    return x / 180.0 * PI;
}

float wrap180(const float x) {
    return x < -180.0f ? (x + 360.0f) : (x > 180.0f ? (x - 360.0f) : x);
}

void wrap2PI(float &x) {
    for (;x >= PI;) 
        x -= 2.0 * PI;
    for (;x < -PI;)
        x += 2.0 * PI;
}

float angle_diff(const float a, const float b) {
    float x = a - b;
    wrap2PI(x);
    return x;
}

float thrust2pwm_dji_set(const float thrust, const float voltage) {
    // Output from matlab:
    /*
     Linear model Poly21:
     a(x,y) = p00 + p10*x + p01*y + p20*x^2 + p11*x*y
       where x is normalized by mean 1525 and std 231.1462
       and where y is normalized by mean 11.7894 and std 0.3300
     Coefficients (with 95% confidence bounds):
       p00 =       2.939  (2.912, 2.966)
       p10 =       2.422  (2.401, 2.442)
       p01 =      0.1464  (0.1259, 0.1669)
       p20 =      0.3989  (0.3762, 0.4215)
       p11 =      0.1119  (0.0913, 0.1325)
    */
    // Reject unreasonable data.
    if (thrust <= 0.0f || voltage <= 7.5f || voltage >= 13.0f) return 1000.0f;
    const float mean_throttle = 1525.0f;
    const float std_throttle = 231.1462f;
    const float mean_voltage = 11.7894f;
    const float std_voltage = 0.3300f;
    const float p00 = 2.939f;
    const float p10 = 2.422f;
    const float p01 = 0.1464f;
    const float p20 = 0.3989f;
    const float p11 = 0.1119f;
    const float y = (voltage - mean_voltage) / std_voltage;
    const float a = p20;
    const float b = p11 * y + p10;
    const float c = p01 * y + p00 - thrust;
    const float delta = b * b - 4 * a * c;
    if (delta < 0.0f) return 1000.0f;
    const float x = (-b + sqrtf(delta)) / 2.0f / a;
    const float throttle = x * std_throttle + mean_throttle;
    return clamp(throttle, 1000.0f, max_pwm);
}
#endif  // AP_MOTORSHybrid
