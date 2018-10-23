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
#include "NN_Controller_Param.h"

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

#define STATE_SIZE  12

#if COPTER_NAME == HYBRID_COPTER
 #define MAX_ROTOR_IN_COPTER 4

const float roll_pitch_degree_max = 20.0;
const float yaw_rate_degree_max = 45.0;
const float altitude_rate_max = 2.0;
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
float deg2rad(const float x);
void constrain_angle(float& x);
float angle_diff(const float x, const float y);

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

float deg2rad(const float x) {
    return x / 180.0 * PI;
}

void wrap2PI(float &x) {
    for (;x >= PI;) 
        x -= 2.0 * PI;
    for (;x < -PI;)
        x += 2.0 * PI;
}

float angle_diff(const float x, const float y) {
    float res = x - y;
    constrain_angle(res);
    return res;
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
            initialization_finished = false;
            initial_yaw_sum = 0.0;
            yaw_count = 0;
            for (i = 0;i < 12;++i) {
                int_diff[i] = 0;
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
            /*if (!initialization_finished) {
                for (i = 0;i < MAX_ROTOR_IN_COPTER;i++)
                    motor_out[i] = 1100;
            } else {
                for (i=0; i<MAX_ROTOR_IN_COPTER; i++) {
                    float pwm = thrust2pwm_dji_set(_thrust_rpyt_out[i], average_voltage);
                    pwm = clamp(pwm, (float)1100.0f, (float)1900.0f);
                    motor_out[i] = (int16_t)pwm;
                }
            }*/
            for (i=0; i<MAX_ROTOR_IN_COPTER; i++) {
                    float pwm = thrust2pwm_dji_set(_thrust_rpyt_out[i], average_voltage);
                    pwm = clamp(pwm, (float)1100.0f, (float)1900.0f);
                    motor_out[i] = (int16_t)pwm;
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

void AP_MotorsHybrid::run_NN_controller(float input[], float output[]) {
    float forward_res[MAX_HIDDEN_LAYER_SIZE];
    for (int i = 0;i < NUM_HIDDEN_LAYERS + 1;++i) {
        int input_size, output_size;
        float tmp_res[MAX_HIDDEN_LAYER_SIZE];
        
        if (i == 0) {
            input_size = STATE_SIZE;
        } else {
            input_size = hidden_layer_size[i - 1];
        }

        if (i == NUM_HIDDEN_LAYERS) {
            output_size = MAX_ROTOR_IN_COPTER;
        } else {
            output_size = hidden_layer_size[i];
        }

        for (int k = 0;k < output_size;++k) {
            tmp_res[k] = B[i][k];
            for (int j = 0;j < input_size;++j) {
                tmp_res[k] += forward_res[j] * W[i][j][k];
            }
        }

        if (i < NUM_HIDDEN_LAYERS) {
            if (ACTIVATION_FUNC == 1) {
                // tanh
            } else (ACTIVATION_FUNC == 2) {
                // relu
            }
        }

        for (int j = 0;j < output_size;++j) {
            forward_res[j] = tmp_res[j];
        }
    }

    for (int i = 0;i < MAX_ROTOR_IN_COPTER;++i) {
        output[i] = forward_res[i];
    }
}

void AP_MotorsHybrid::output_armed_stabilizing() {
    // estimate voltage
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
    const float z = 0;
    const float roll = _copter.get_roll();
    const float pitch = _copter.get_pitch();
    const float yaw = 0;
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
    const float z0 = 0;
    const float thr_ctrl = (float)_copter.get_channel_throttle_control_in();
    const float roll_ctrl = (float)_copter.get_channel_roll_control_in();
    const float pitch_ctrl = (float)_copter.get_channel_pitch_control_in();
    const float yaw_ctrl = (float)_copter.get_channel_yaw_control_in();

    float vz0 = 0.0f;
    if (thr_ctrl < 400.0f) {
        vz0 = remap(thr_ctrl, 0.0f, 400.0f, -altitude_rate_max, 0.0f);
    } else if (thr_ctrl > 600.0f) {
        vz0 = remap(thr_ctrl, 600.0f, 1000.0f, 0.0f, altitude_rate_max);
    } else {
        vz0 = 0.0f;
    }

    // remap roll, pitch, yaw control input => degrees
    float roll0, pitch0;
    if (roll_ctrl < -500.0f) {
        roll0 = remap(roll_ctrl, -4500.0f, -500.0f, -deg2rad(roll_pitch_degree_max), 0.0f);
    } else if (roll_ctrl > 500.0f) {
        roll0 = remap(roll_ctrl, 500.0f, 4500.0f, 0.0f, deg2rad(roll_pitch_degree_max));
    } else {
        roll0 = 0.0f;
    }
    if (pitch_ctrl < -500.0f) {
        pitch0 = remap(pitch_ctrl, -4500.0f, -500.0f, -deg2rad(roll_pitch_degree_max), 0.0f);
    } else if (pitch_ctrl > 500.0f) {
        pitch0 = remap(pitch_ctrl, 500.0f, 4500.0f, 0.0f, deg2rad(roll_pitch_degree_max));
    } else {
        pitch0 = 0.0f;
    }
    
    float yaw_change_deadzone = 500;
    float desired_yaw_rate;
    if (fabs(yaw_ctrl) < yaw_change_deadzone)
        desired_yaw_rate = 0.0;
    else if (yaw_ctrl > 0)
        desired_yaw_rate = remap(yaw_ctrl, yaw_change_deadzone, 4500.0, 0.0, deg2rad(yaw_rate_degree_max));
    else
        desired_yaw_rate = remap(yaw_ctrl, -4500.0, -yaw_change_deadzone, -deg2rad(yaw_rate_degree_max), 0.0);

    const float X0[NUM_COL] = {0, 0, 0.0, roll0, pitch0, 0.0f, 0.0f, 0.0f, -vz0, 0.0f, 0.0f, desired_yaw_rate};

    // Compute the controller input
    float X_minus_X0[NUM_COL];
    for (int i = 0; i < NUM_COL; ++i) {
        X_minus_X0[i] = X[i] - X0[i];
    }
    // Take care of the yaw difference so that the abs value of the difference is smaller
    // than pi.
    for (int i = 3;i < 6;++i) {
        wrap2PI(X_minus_X0[i]);
    }

    run_NN_controller(X_minus_X0, _thrust_rpyt_out);

    // save info to copter
    
    _copter.real_x = x; _copter.real_y = y; _copter.real_z = z;
    _copter.real_roll = roll; _copter.real_pitch = pitch; _copter.real_yaw = yaw;
    _copter.real_vx = vx; _copter.real_vy = vy; _copter.real_vz = vz;
    _copter.real_rollspeed = rollspeed; _copter.real_pitchspeed = pitchspeed; _copter.real_yawspeed = yawspeed;
    _copter.desired_vz = vz0;
    _copter.desired_roll = roll0;
    _copter.desired_pitch = pitch0;
    _copter.desired_yaw_rate = desired_yaw_rate;
    _copter.desired_thrust[0] = _thrust_rpyt_out[0]; 
    _copter.desired_thrust[1] = _thrust_rpyt_out[1];
    _copter.desired_thrust[2] = _thrust_rpyt_out[2];
    _copter.desired_thrust[3] = _thrust_rpyt_out[3];
    _copter.thr_ctrl_in = (int16_t)thr_ctrl;
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