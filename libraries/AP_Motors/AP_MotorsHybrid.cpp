/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_MotorsMatrix.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 */
 #include "../../ArduCopter/Copter.h"
#include <AP_HAL/AP_HAL.h>
#include "AP_MotorsHybrid.h"

#define QUAD_ROTOR  1
#define FIVE_ROTOR  2
#define BUNNY_ROTOR 3

#define COPTER_NAME       HYBRID_COPTER

// The meaning of each column
#define X_COL       0
#define Y_COL       1
#define Z_COL       2
#define ROL_COL     3
#define PIT_COL     4
#define YAW_COL     5
#define X_VEL_COL   6
#define Y_VEL_COL   7
#define Z_VEL_COL   8
#define ROL_VEL_COL 9
#define PIT_VEL_COL 10
#define YAW_VEL_COL 11
#define NUM_COL     12

#if COPTER_NAME == HYBRID_COPTER
 #define MAX_ROTOR_IN_COPTER 4
// Q = [1 1 1 1 1 1 1 1 1 1 1 1]. R = [5 5 5 5]. Mass = 1.061kg.
const float K[MAX_ROTOR_IN_COPTER][NUM_COL] = {
{0.001813f,  -0.315882f,  -0.224487f,  -2.802199f,  0.000131f,  0.315660f,  0.002959f,  -0.616608f,  -0.500556f,  -0.654212f,  0.015672f,  0.843064f,  },
{0.001644f,  0.316570f,  -0.222716f,  2.807963f,  0.000696f,  0.316791f,  0.002680f,  0.617939f,  -0.496597f,  0.655216f,  0.015655f,  0.846076f,  },
{-0.316049f,  0.000827f,  -0.224952f,  0.006889f,  2.541661f,  -0.314672f,  -0.603366f,  0.001591f,  -0.501588f,  0.001505f,  0.526616f,  -0.839463f,  },
{0.316397f,  0.000991f,  -0.222260f,  0.009187f,  -2.572984f,  -0.317780f,  0.604988f,  0.001955f,  -0.495580f,  0.002219f,  -0.564177f,  -0.849685f,  },
};
const float u0[MAX_ROTOR_IN_COPTER] = {
    3.269005f,
    3.243095f,
    3.275689f,
    3.236411f,
};

const float xybound = 3.0f;
const float lower_z = 7.0f;
const float upper_z = -3.0f;
const float max_pwm = 2000.0f;
#endif

// For voltage estimation.
static int last_frame = 0;
static int current_frame = 0;
static float average_voltage = 0.0f;
static float voltage_sum = 0.0f;

extern const AP_HAL::HAL& hal;

float remap(const float in_value, const float in_low, const float in_high,
        const float out_low, const float out_high);
float clamp(const float in_value, const float in_low, const float in_high);
float wrap180(const float x);
float thrust2pwm_kde_14inch(const float thrust, const float voltage);
float thrust2pwm_kde_10inch(const float thrust, const float voltage);
float thrust2pwm_black_bi(const float thrust, const float voltage);

// Definitions of helper functions.
float remap(const float in_value, const float in_low, const float in_high,
        const float out_low, const float out_high) {
    if (fabs(in_low - in_high) < 1e-5) return (out_low + out_high) / 2.0f;
    return (in_value - in_low) / (in_high - in_low) * (out_high - out_low) + out_low;
}

float clamp(const float in_value, const float in_low, const float in_high) {
    return (in_value < in_low ? in_low : (in_value > in_high ? in_high : in_value));
}

float wrap180(const float x) {
    return x < -180.0f ? (x + 360.0f) : (x > 180.0f ? (x - 360.0f) : x);
}

float thrust2pwm_kde_14inch(const float thrust, const float voltage) {
    // Reject unreasonable data.
    if (thrust <= 0.0f || voltage <= 7.5f || voltage >= 13.0f) return 1000.0f;
    // Output from matlab:
    /*
    Linear model Poly21:
    a(x,y) = p00 + p10*x + p01*y + p20*x^2 + p11*x*y
      where x is normalized by mean 1425 and std 201.8
      and where y is normalized by mean 11.38 and std 0.3678
    Coefficients (with 95% confidence bounds):
      p00 =         3.4  (3.375, 3.424)
      p10 =       3.324  (3.306, 3.341)
      p01 =      0.2254  (0.2078, 0.2431)
      p20 =      0.7846  (0.7648, 0.8043)
      p11 =      0.1704  (0.1526, 0.1883)
    */
    const float mean_throttle = 1425.0f;
    const float std_throttle = 201.8140f;
    const float mean_voltage = 11.3775f;
    const float std_voltage = 0.3678f;
    const float p00 = 3.4f;
    const float p10 = 3.324f;
    const float p01 = 0.2254f;
    const float p20 = 0.7846f;
    const float p11 = 0.1704f;
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

float thrust2pwm_kde_10inch(const float thrust, const float voltage) {
    // Output from matlab:
    /*
     Linear model Poly21:
     a(x,y) = p00 + p10*x + p01*y + p20*x^2 + p11*x*y
       where x is normalized by mean 1550 and std 274.1
       and where y is normalized by mean 11.48 and std 0.3972
     Coefficients (with 95% confidence bounds):
       p00 =       2.827  (2.805, 2.848)
       p10 =       2.037  (2.023, 2.052)
       p01 =      0.1959  (0.1812, 0.2105)
       p20 =      0.1797  (0.1633, 0.196)
       p11 =      0.1362  (0.1215, 0.151)
    */
    // Reject unreasonable data.
    if (thrust <= 0.0f || voltage <= 7.5f || voltage >= 13.0f) return 1000.0f;
    const float mean_throttle = 1550.0f;
    const float std_throttle = 274.1f;
    const float mean_voltage = 11.48f;
    const float std_voltage = 0.3972f;
    const float p00 = 2.827f;
    const float p10 = 2.037f;
    const float p01 = 0.1959f;
    const float p20 = 0.1797f;
    const float p11 = 0.1362f;
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

float thrust2pwm_black_bi(const float thrust, const float voltage) {
    // Output from matlab:
    /*
     Linear model Poly21:
     a(x,y) = p00 + p10*x + p01*y + p20*x^2 + p11*x*y
       where x is normalized by mean 1525 and std 231.9428
       and where y is normalized by mean 11.2309 and std 0.2273
     Coefficients (with 95% confidence bounds):
       p00 =       1.577  (1.56, 1.594)
       p10 =       1.405  (1.377, 1.433)
       p01 =     0.01139  (-0.01619, 0.03897)
       p20 =      0.2806  (0.2549, 0.3062)
       p11 =     0.03498  (0.01467, 0.0553)
    */
    // Reject unreasonable data.
    if (thrust <= 0.0f || voltage <= 7.5f || voltage >= 13.0f) return 1000.0f;
    const float mean_throttle = 1525.0f;
    const float std_throttle = 231.9428f;
    const float mean_voltage = 11.2309f;
    const float std_voltage = 0.2273f;
    const float p00 = 1.577f;
    const float p10 = 1.405f;
    const float p01 = 0.01139f;
    const float p20 = 0.2806f;
    const float p11 = 0.03498f;
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

void AP_MotorsHybrid::setup_motors(motor_frame_class frame_class, motor_frame_type frame_type) {
    // call parent
    AP_MotorsMatrix::setup_motors(frame_class, frame_type);

    // Add at most 6 motors. The roll/pitch/yaw factor does not really matter
    // as we are going to send desired pwm via mavlink.
    for (int i = 0; i < MAX_ROTOR_IN_COPTER; ++i) {
        add_motor_raw(AP_MOTORS_MOT_1 + i, 0.0f, 0.0f, 0.0f, i + 1);
    }
}

void AP_MotorsHybrid::output_to_motors() {
    int8_t i;
    int16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];    // final pwm values sent to the motor

    // Increment current frame.
    ++current_frame;
    // Compute the mean voltage since last update. We take the mean voltage during the last
    // 2 seconds. Note that this function gets called at 400Hz.
    if (current_frame - last_frame > 400 * 2) {
        average_voltage = voltage_sum / (current_frame - last_frame);
        voltage_sum = 0.0f;
        last_frame = current_frame;
    } else {
        voltage_sum += _copter.get_battery_voltage();
    }
    // estimate voltage by _batt_voltage_filt.get() * _batt_voltage_max

    switch (_spool_mode) {
        case SHUT_DOWN: {
            // sends minimum values out to the motors
            // set motor output based on thrust requests
            for (i=0; i<MAX_ROTOR_IN_COPTER; i++) {
                if (_disarm_disable_pwm && _disarm_safety_timer == 0 && !armed()) {
                    motor_out[i] = 0;
                } else {
                    motor_out[i] = get_pwm_output_min();
                }
            }
            break;
        }
        case SPIN_WHEN_ARMED:
            // sends output to motors when armed but not flying
            /*for (i=0; i<MAX_ROTOR_IN_COPTER; i++) {
                motor_out[i] = calc_spin_up_to_pwm();
            }
            break;*/
        case SPOOL_UP:
        case THROTTLE_UNLIMITED:
        case SPOOL_DOWN:
            // set motor output based on thrust requests
            for (i=0; i<MAX_ROTOR_IN_COPTER; i++) {
                float pwm = thrust2pwm_black_bi(_thrust_rpyt_out[i], average_voltage);
                pwm = clamp(pwm, (float)1100.0f, (float)1900.0f);
                motor_out[i] = (int16_t)pwm;
                //double angle = (current_frame % 360 + i * 60) / 180.0 * 3.14159265;
                //motor_out[i] = 400.0 * sin(angle) + 1500.0f;
                // motor_out[i] = 1500;
            }
            break;
    }

    // send output to each motor
    for (i=0; i<MAX_ROTOR_IN_COPTER; i++) {
        rc_write(i, motor_out[i]);
    }
    _copter.pwm_out[0] = motor_out[0]; _copter.pwm_out[1] = motor_out[1]; _copter.pwm_out[2] = motor_out[2]; _copter.pwm_out[3] = motor_out[3];
    _copter.real_battery = average_voltage;
    _copter.spool_mode = (int)_spool_mode;
}

void AP_MotorsHybrid::output_armed_stabilizing() {
    {
        float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0

        throttle_thrust = get_throttle() * get_compensation_gain();

        // sanity check throttle is above zero and below current limited throttle
        if (throttle_thrust <= 0.0f) {
            throttle_thrust = 0.0f;
            limit.throttle_lower = true;
        }
        if (throttle_thrust >= _throttle_thrust_max) {
            throttle_thrust = _throttle_thrust_max;
            limit.throttle_upper = true;
        }
    }
    // Get current states:
    const float x = 0;
    const float y = 0;
    const float z = -_copter.get_altitude(); // get altitude
    const float roll = _copter.get_roll();
    const float pitch = _copter.get_pitch();
    const float yaw = 0;//_copter.get_yaw();
    // get velocities
    const Vector3f velocity_ef = _copter.get_ned_velocity();
    const Vector3f velocity_bf = _copter.frame_conversion_ef_to_bf(velocity_ef);
    const float vx = velocity_bf.x;
    const float vy = velocity_bf.y;
    const float vz = velocity_bf.z;
    const float rollspeed = _copter.get_roll_rate();
    const float pitchspeed = _copter.get_pitch_rate();
    const float yawspeed = _copter.get_yaw_rate();
    const float X[NUM_COL] = {x, y, z, roll, pitch, yaw, vx, vy, vz, rollspeed, pitchspeed, yawspeed};

    // Get desired states.
    const float x0 = 0;
    const float y0 = 0;
    const float thr_ctrl = (float)_copter.get_channel_throttle_control_in();
    float z0 = 0.0f;
    if (thr_ctrl < 500.0f) {
        z0 = remap(thr_ctrl, 0.0f, 500.0f, lower_z, 0.0f);
    } else {
        z0 = remap(thr_ctrl, 500.0f, 1000.0f, 0.0f, upper_z);
    }
    const float X0[NUM_COL] = {x0, y0, z0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    // Compute the desired thrust.
    // u = -K(X - X0) + u0.
    float X_minus_X0[NUM_COL];
    for (int i = 0; i < NUM_COL; ++i) {
        X_minus_X0[i] = X[i] - X0[i];
    }
    // Take care of the yaw difference so that the abs value of the difference is smaller
    // than pi.
    float yaw_diff = X_minus_X0[5];
    if (yaw_diff > PI) {
        yaw_diff -= 2 * PI;
    } else if (yaw_diff < -PI) {
        yaw_diff += 2 * PI;
    }
    X_minus_X0[5] = 0;//yaw_diff;

    float K_times_X_minus_X0[MAX_ROTOR_IN_COPTER];
    for (int i = 0; i < MAX_ROTOR_IN_COPTER; ++i) {
        K_times_X_minus_X0[i] = 0.0f;
        for (int j = 0; j < NUM_COL; ++j) {
            K_times_X_minus_X0[i] += K[i][j] * X_minus_X0[j];
        }
    }
    for (int i = 0; i < MAX_ROTOR_IN_COPTER; ++i) {
        _thrust_rpyt_out[i] = -K_times_X_minus_X0[i] + u0[i];
    }

    // save info to copter
    _copter.real_x = x; _copter.real_y = y; _copter.real_z = z;
    _copter.real_roll = roll; _copter.real_pitch = pitch; _copter.real_yaw = yaw;
    _copter.real_vx = vx; _copter.real_vy = vy; _copter.real_vz = vz;
    _copter.real_rollspeed = rollspeed; _copter.real_pitchspeed = pitchspeed; _copter.real_yawspeed = yawspeed;
    _copter.desired_z = z0;
    _copter.desired_thrust[0] = _thrust_rpyt_out[0]; 
    _copter.desired_thrust[1] = _thrust_rpyt_out[1];
    _copter.desired_thrust[2] = _thrust_rpyt_out[2];
    _copter.desired_thrust[3] = _thrust_rpyt_out[3];
    _copter.throttle_in = (int16_t)thr_ctrl;
}

/*
  call vehicle supplied thrust compensation if set. This allows
  vehicle code to compensate for vehicle specific motor arrangements
  such as tiltrotors or tiltwings
*/
void AP_MotorsHybrid::thrust_compensation(void)
{
    if (_thrust_compensation_callback) {
        _thrust_compensation_callback(_thrust_rpyt_out, AP_MOTORS_MAX_NUM_MOTORS);
    }
}