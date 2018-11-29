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

// The meaning of each column
#define ANGLE_AXIS_X        0
#define ANGLE_AXIS_Y        1
#define ANGLE_AXIS_Z        2
#define V_X                 3
#define V_Y                 4
#define V_Z                 5
#define OMEGA_X             6
#define OMEGA_Y             7
#define OMEGA_Z             8
#define TARGET_VX           9
#define TARGET_VY           10

#define NUM_STATE           9
#define NUM_INPUT           11
#define NUM_ROTORS          4

const float min_vx = 0.0f;
const float max_vx = 6.0f;
const float min_vz = -1.0f;
const float max_vz = 1.0f;
const float max_pwm = 2000.0f;

// For voltage estimation.
static int last_frame = 0;
static int current_frame = 0;
static float average_voltage = 0.0f;
static float voltage_sum = 0.0f;

extern const AP_HAL::HAL& hal;

// Definitions of helper functions.
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

void AP_MotorsHybrid::setup_motors(motor_frame_class frame_class, motor_frame_type frame_type) {
    // call parent
    AP_MotorsMatrix::setup_motors(frame_class, frame_type);

    // Add at most 6 motors. The roll/pitch/yaw factor does not really matter
    // as we are going to send desired pwm via mavlink.
    for (int i = 0; i < NUM_ROTORS; ++i) {
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
            for (i=0; i<NUM_ROTORS; i++) {
                if (_disarm_disable_pwm && _disarm_safety_timer == 0 && !armed()) {
                    motor_out[i] = 0;
                } else {
                    motor_out[i] = get_pwm_output_min();
                }
            }
            initialization_finished = false;
            initial_yaw_sum = 0.0;
            yaw_count = 0;
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
            if (!initialization_finished) {
                for (i = 0;i < NUM_ROTORS;i++)
                    motor_out[i] = 1100;
            } else {
                for (i=0; i<NUM_ROTORS; i++) {
                    float pwm = thrust2pwm_dji_set(_thrust_rpyt_out[i], average_voltage);
                    pwm = clamp(pwm, (float)1100.0f, (float)1900.0f);
                    motor_out[i] = (int16_t)pwm;
                }
            }
            break;
    }

    // send output to each motor
    for (i=0; i<NUM_ROTORS; i++) {
        rc_write(i, motor_out[i]);
    }
    for (i = 0;i < NUM_ROTORS; i++) {
        _copter.desired_thrust[i] = _thrust_rpyt_out[i];
    }
    _copter.real_battery = average_voltage;
    _copter.spool_mode = (int)_spool_mode;
}

void AP_MotorsHybrid::get_observation_vector(float ob[]) {
    float state[NUM_STATE];
    get_state(state);
    for (int i = 0;i < NUM_STATE;i++) {
        ob[i] = state[i];
    }
    ob[NUM_STATE] = target_vx;
    ob[NUM_STATE + 1] = target_vz;
}

void AP_MotorsHybrid::get_state(float state[]) {
    collect_rpy();

    float angle_axis[3];
    get_angle_axis(angle_axis);

    float vel[3];
    get_velocity(vel);

    float omega[3];
    get_angular_velocity(omega);

    state[0] = angle_axis[0]; state[1] = angle_axis[1]; state[2] = angle_axis[2];
    state[3] = vel[0]; state[4] = vel[1]; state[5] = vel[2];
    state[6] = omega[0]; state[7] = omega[1]; state[8] = omega[2];
}

void AP_MotorsHybrid::collect_rpy() {
    roll = _copter.get_roll();
    pitch = _copter.get_pitch();
    yaw = _copter.get_yaw() - yaw_0;
}

Matrix3f AP_MotorsHybrid::get_rotation_matrix() {    
    Matrix3f R_roll, R_pitch, R_yaw;
    
    R_roll[0][0] = 1; R_roll[0][1] = 0; R_roll[0][2] = 0;
    R_roll[1][0] = 0; R_roll[1][1] = cos(roll); R_roll[1][2] = -sin(roll);
    R_roll[2][0] = 0; R_roll[2][1] = sin(roll); R_roll[2][2] = cos(roll);

    R_pitch[0][0] = cos(pitch); R_pitch[0][1] = 0; R_pitch[0][2] = sin(pitch);
    R_pitch[1][0] = 0; R_pitch[1][1] = 1; R_pitch[1][2] = 0;
    R_pitch[2][0] = -sin(pitch); R_pitch[2][1] = 0; R_pitch[2][2] = cos(pitch);

    R_yaw[0][0] = cos(yaw); R_yaw[0][1] = -sin(yaw); R_yaw[0][2] = 0;
    R_yaw[1][0] = sin(yaw); R_yaw[1][1] = cos(yaw); R_yaw[1][2] = 0;
    R_yaw[2][0] = 0; R_yaw[2][1] = 0; R_yaw[2][2] = 1;

    return R_yaw * R_pitch * R_roll;
}

void AP_MotorsHybrid::get_angle_axis(float angle_axis[]) {
    // first convert rpy to unit quaternion
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);

    float w = cy * cr * cp + sy * sr * sp;
    float x = cy * sr * cp - sy * cr * sp;
    float y = cy * cr * sp + sy * sr * cp;
    float z = sy * cr * cp - cy * sr * sp;

    float len = sqrt(w * w + x * x + y * y + z * z);
    w /= len; x /= len; y /= len; z /= len;

    // convert unit quaternion to angle axis
    float angle = 2.0 * acos(w);
    wrap2PI(angle);

    float a = sqrt(1.0 - w * w);
    float axis_x = x / a;
    float axis_y = y / a;
    float axis_z = z / a;

    angle_axis[0] = axis_x * a;
    angle_axis[1] = axis_y * a;
    angle_axis[2] = axis_z * a;
}

void AP_MotorsHybrid::get_velocity(float vel[]) {
    // coordination 0 (ned) rotate yaw to coordination 1 (our ned)
    // R*v1 = v0
    // v1 = R^T*v0
    // R(yaw) = 
    // [cos, -sin, 0]
    // [sin,  cos, 0]
    // [0,    0,   1]
    const Vector3f velocity_ef = _copter.get_ned_velocity();
    vel[0] = velocity_ef[0] * cos(yaw_0) + velocity_ef[1] * sin(yaw_0);
    vel[1] = - velocity_ef[0] * sin(yaw_0) + velocity_ef[1] * cos(yaw_0);
    vel[2] = velocity_ef[2];
}

/*
void AP_MotorsHybrid::get_angular_velocity(float omega[]) {
    // first convert roll_rate, pitch_rate, yaw_rate into body-frame angular velocity p_b, q_b, r_b
    // based on MAV331 Lec9
    // [p, q, r]' = [1, 0,           -sin(pitch)
    //               0, cos(roll),   sin(roll)*cos(pitch)
    //               0, -sin(roll),  cos(roll)*cos(pitch)]
    // * [roll_rate, pitch_rate, yaw_rate]'
    Matrix3f rpy_rate_to_pqr_matrix;
    rpy_rate_to_pqr_matrix[0][0] = 1; rpy_rate_to_pqr_matrix[0][1] = 0; rpy_rate_to_pqr_matrix[0][2] = -sin(pitch);
    rpy_rate_to_pqr_matrix[1][0] = 0; rpy_rate_to_pqr_matrix[1][1] = cos(roll); rpy_rate_to_pqr_matrix[1][2] = sin(roll) * cos(pitch);
    rpy_rate_to_pqr_matrix[2][0] = 0; rpy_rate_to_pqr_matrix[2][1] = -sin(roll); rpy_rate_to_pqr_matrix[2][2] = cos(roll) * cos(pitch);

    Vector3f rpy_rate(_copter.get_roll_rate(), _copter.get_pitch_rate(), _copter.get_yaw_rate());
    Vector3f pqr_body = rpy_rate_to_pqr_matrix * rpy_rate;
    
    // then convert pqr body to pqr world
    Vector3f pqr_world = get_rotation_matrix() * pqr_body;
}*/

void AP_MotorsHybrid::get_angular_velocity(float omega[]) {
    Vector3f omega_body = _copter.get_omega_body();
    Vector3f _omega = get_rotation_matrix() * omega_body;
    omega[0] = _omega.x;
    omega[1] = _omega.y;
    omega[2] = _omega.z;
}

void AP_MotorsHybrid::output_armed_stabilizing() {
    {
        if (!initialization_finished) {
            float now_yaw = _copter.get_yaw();
            if (yaw_count == 0) {
                initial_yaw_sum = now_yaw;
                yaw_count = 1;
            } else {
                float estimated_initial_yaw = (float)(initial_yaw_sum / yaw_count);
                if (angle_diff(estimated_initial_yaw, now_yaw) > deg2rad(30)) {
                    initial_yaw_sum = now_yaw;
                    yaw_count = 1;
                } else {
                    if (fabs(estimated_initial_yaw - now_yaw) > deg2rad(180)) {
                        if (estimated_initial_yaw > now_yaw)
                            now_yaw += 2.0 * PI;
                        else
                            now_yaw -= 2.0 * PI;
                    }
                    initial_yaw_sum += now_yaw;
                    yaw_count ++;
                }
            }
            if (yaw_count == 400) {
                yaw_0 = (float)(initial_yaw_sum / yaw_count);
                wrap2PI(initial_yaw);
                desired_yaw = initial_yaw;
                initialization_finished = true;
            }
        }
    }

    // Ardupilot's code
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

    // calculate the desired vx and vz
    const float thr_ctrl = (float)_copter.get_channel_throttle_control_in();
    const float pitch_ctrl = (float)_copter.get_channel_pitch_control_in();
    target_vz = 0.0f;
    if (thr_ctrl < 400.0f) {
        target_vz = remap(thr_ctrl, 0.0f, 400.0f, min_vz, 0.0f);
    } else if (thr_ctrl > 600.0f) {
        target_vz = remap(thr_ctrl, 600.0f, 1000.0f, 0.0f, max_vz);
    } else {
        target_vz = 0;
    }
    target_vx = 0.0f;
    if (pitch_ctrl < 500.0f) {
        target_vx = 0.0f;
    } else {
        target_vx = remap(pitch_ctrl, 500.0f, 4500.0f, 0.0f, max_vx);
    } 

    // get NN input
    float input[NUM_INPUT];
    get_observation_vector(input);

    // Compute the desired thrust.
    for (int i = 0;i < NUM_ROTORS;i ++) {
        _thrust_rpyt_out[i] = 0;
    }

    // save info to copter
    _copter.angle_axis[0] = input[0]; _copter.angle_axis[1] = input[1]; _copter.angle_axis[2] = input[2];
    _copter.vel[0] = input[3]; _copter.vel[1] = input[4]; _copter.vel[2] = input[5];
    _copter.omega[0] = input[6]; _copter.omega[1] = input[7]; _copter.omega[2] = input[8];
    _copter.target_vx = target_vx; _copter.target_vz = target_vz;
    for (int i = 0;i < NUM_ROTORS;i++) {
        _copter.desired_thrust[i] = _thrust_rpyt_out[i];
    }
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