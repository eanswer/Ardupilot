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
#include "AP_MotorsHybridHelper.h"
#include "AP_MotorsPolicyDefinition.h"

// For voltage estimation.
static int last_frame = 0;
static int current_frame = 0;
static float average_voltage = 0.0f;
static float voltage_sum = 0.0f;

extern const AP_HAL::HAL& hal;

void AP_MotorsHybrid::set_radios_switch(uint16_t switch_CH6) {
    if (switch_CH6 < 1300) {
        mode = 0;
    } else {
        mode = 1;
    }
}

void AP_MotorsHybrid::setup_motors(motor_frame_class frame_class, motor_frame_type frame_type) {
    // call parent
    AP_MotorsMatrix::setup_motors(frame_class, frame_type);

    // Add at most 6 motors. The roll/pitch/yaw factor does not really matter
    // as we are going to send desired pwm via mavlink.
    // for (int i = 0; i < AC_SPACE_SIZE; ++i) {
    //     add_motor_raw(AP_MOTORS_MOT_1 + i, 0.0f, 0.0f, 0.0f, i + 1);
    // }
}

void AP_MotorsHybrid::output_to_motors() {
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

    _copter.real_battery = average_voltage;
    _copter.policy_mode = mode;

    if (!initialization_finished) {
        for (int i = 0;i < AC_SPACE_SIZE;i++)
            _motor_out_NN[i] = _motor_out_pid[i] = 1100;
    } else {
        if (RUN_PID == 1) {
            AP_MotorsMatrix::output_to_motors();
        }

        if (RUN_NN == 1) {
            int8_t i;
            int16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];    // final pwm values sent to the motor

            switch (_spool_mode) {
                case SHUT_DOWN: {
                    // sends minimum values out to the motors
                    // set motor output based on thrust requests
                    for (i=0; i<AC_SPACE_SIZE; i++) {
                        if (_disarm_disable_pwm && _disarm_safety_timer == 0 && !armed()) {
                            motor_out[i] = 0;
                        } else {
                            motor_out[i] = get_pwm_output_min();
                        }
                    }
                    initialization_finished = false;
                    initial_yaw_sum = 0.0;
                    yaw_count = 0;
                    target_yaw_diff = 0.0f;
                    last_omega[0] = last_omega[1] = last_omega[2] = 0;
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
                        for (i = 0;i < AC_SPACE_SIZE;i++)
                            motor_out[i] = 1100;
                    } else {
                        for (i=0; i<AC_SPACE_SIZE; i++) {
                            float pwm = thrust2pwm_dji_set(_thrust_rpyt_out_NN[i], average_voltage);
                            pwm = clamp(pwm, (float)1100.0f, (float)1900.0f);
                            motor_out[i] = (int16_t)pwm;
                        }
                    }
                    break;
            }

            for (i = 0;i < AC_SPACE_SIZE;i++) {
                _motor_out_NN[i] = motor_out[i];
            }
        }
    }

    for (int i = 0;i < AC_SPACE_SIZE;i++) {
        _copter.motor_out_pid[i] = _motor_out_pid[i];
        _copter.motor_out_NN[i] = _motor_out_NN[i];
    }
    
    // send output to each motor
    if (mode == 0) {
        for (int i = 0; i < AC_SPACE_SIZE; i++) {
            rc_write(i, _motor_out_NN[i]);
            // float output = _motor_out_pid[i] * 0.2 + _motor_out_NN[i] * 0.8;
            // rc_write(i, (int16_t)output);
        }
    } else {
        for (int i = 0;i < AC_SPACE_SIZE; i++) {
            rc_write(i, _motor_out_pid[i]);
        }
    }
    _copter.spool_mode = (int)_spool_mode;
}

void AP_MotorsHybrid::get_observation_vector(float ob[]) {
    float state[STATE_SIZE];
    get_state(state);
    for (int i = 0;i < STATE_SIZE;i++) {
        ob[i] = state[i];
    }
    ob[STATE_SIZE] = target_vx;
    ob[STATE_SIZE + 1] = target_vy;
    ob[STATE_SIZE + 2] = target_vz;
}

void AP_MotorsHybrid::get_state(float state[]) {
    collect_rpy();

    float vel[3];
    get_velocity_body(vel);

    float omega[3];
    get_angular_velocity(omega);

    state[0] = roll; state[1] = pitch; state[2] = yaw - target_yaw_diff;
    state[3] = vel[0]; state[4] = vel[1]; state[5] = vel[2];
    state[6] = omega[0]; state[7] = omega[1]; state[8] = omega[2];

    wrap2PI(state[2]);

    state[2] = clamp(state[2], -0.1, 0.1);
    for (int i = 3;i < 9;i++) {
        state[i] = clamp(state[i], -1.5, 1.5);
    }
}

void AP_MotorsHybrid::collect_rpy() {
    roll = _copter.get_roll();
    pitch = _copter.get_pitch();
    yaw = _copter.get_yaw() - yaw_0;
    wrap2PI(yaw);
}

Matrix3f AP_MotorsHybrid::get_rotation_matrix() {    
    // another option here is: ahrs.get_rotation_body_to_ned()
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

    Matrix3f rotation_matrix = R_yaw * R_pitch * R_roll;
    
    // const Matrix3f rotation_matrix_from_ahrs = _copter.get_rotation_matrix();
    // float diff = 0;
    // for (int i = 0;i < 3;i++)
    //     for (int j = 0;j < 3;j++)
    //         diff += (rotation_matrix[i][j] - rotation_matrix_from_ahrs[i][j]) * (rotation_matrix[i][j] - rotation_matrix_from_ahrs[i][j]);
    
    // _copter.rotation_matrix_diff = diff;

    return rotation_matrix;
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

// local NED frame
void AP_MotorsHybrid::get_velocity_body(float vel[]) {
    const Vector3f velocity_ef = _copter.get_ned_velocity();
    vel[0] = velocity_ef[0] * _copter.get_cos_yaw() + velocity_ef[1] * _copter.get_sin_yaw();
    vel[1] = - velocity_ef[0] * _copter.get_sin_yaw() + velocity_ef[1] * _copter.get_cos_yaw();
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
    /*Vector3f _omega = get_rotation_matrix() * omega_body;
    omega[0] = _omega.x;
    omega[1] = _omega.y;
    omega[2] = _omega.z;*/
    omega[0] = omega_body[0];
    omega[1] = omega_body[1];
    omega[2] = omega_body[2];
}

void AP_MotorsHybrid::pi_act(float ob[], float action[]) {
    // run normalization
    float ob_normalized[OB_SPACE_SIZE];
    for (int i = 0;i < OB_SPACE_SIZE;i++) {
        ob_normalized[i] = (ob[i] - ob_mean[i]) / ob_std[i];
    }

    // run mlp policy
    float last_out[HIDDEN_LAYER_SIZE];
    float tmp[HIDDEN_LAYER_SIZE];

    for (int j = 0;j < HIDDEN_LAYER_SIZE;j++) {
        tmp[j] = b0[j];
        for (int i = 0;i < OB_SPACE_SIZE;i++) {
            tmp[j] += ob_normalized[i] * W0[i][j];
        }
    }
    for (int j = 0;j < HIDDEN_LAYER_SIZE;j++) {
        last_out[j] = tanhf(tmp[j]);
    }
    
    for (int i = 0;i < NUM_HIDDEN_LAYER - 1;i++) {
        for (int j = 0;j < HIDDEN_LAYER_SIZE;j++) {
            tmp[j] = b_hidden[i][j];
            for (int k = 0;k < HIDDEN_LAYER_SIZE;k++) {
                tmp[j] += last_out[k] * W_hidden[i][k][j];
            }
        }
        for (int j = 0;j < HIDDEN_LAYER_SIZE;j++) {
            last_out[j] = tanhf(tmp[j]);
        }
    }

    for (int j = 0;j < AC_SPACE_SIZE;j++) {
        tmp[j] = b1[j];
        for (int i = 0;i < HIDDEN_LAYER_SIZE;i ++) {
            tmp[j] += last_out[i] * W1[i][j];
        }
    }
    for (int j = 0;j < AC_SPACE_SIZE;j++) {
        last_out[j] = tmp[j];
    }
    
    for (int j = 0;j < AC_SPACE_SIZE;j++) {
        action[j] = last_out[j] + FINAL_BIAS;
        if (action[j] < 0)
            action[j] = 0;
        if (action[j] > FINAL_BIAS * 2.0)
            action[j] = FINAL_BIAS * 2.0;
    }
}

void AP_MotorsHybrid::output_armed_stabilizing() {
    if (!initialization_finished) {
        float now_yaw = _copter.get_yaw();
        if (yaw_count == 0) {
            initial_yaw_sum = now_yaw;
            yaw_count = 1;
        } else {
            float estimated_initial_yaw = (float)(initial_yaw_sum / yaw_count);
            if (fabs(angle_diff(estimated_initial_yaw, now_yaw)) > deg2rad(30)) {
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
        if (yaw_count == 4000) {
            yaw_0 = (float)(initial_yaw_sum / yaw_count);
            initialization_finished = true;
            target_yaw_diff = 0.0;
            last_omega[0] = last_omega[1] = last_omega[2] = 0;
        }
    }
    
    if (RUN_PID == 1) {
        AP_MotorsMatrix::output_armed_stabilizing();
    }

    if (RUN_NN == 1) {
        // calculate the desired vx and vz
        const float thr_ctrl = (float)_copter.get_channel_throttle_control_in();
        const float pitch_ctrl = (float)_copter.get_channel_pitch_control_in();
        const float roll_ctrl = (float)_copter.get_channel_roll_control_in();
        const float yaw_ctrl = (float)_copter.get_channel_yaw_control_in();

        target_vx = 0.0f;
        if (pitch_ctrl > 500.0f) {
            target_vx = remap(pitch_ctrl, 500.0, 4500.0f, 0.0f, min_vx);
        } else if (pitch_ctrl < -500.0f) {
            target_vx = remap(pitch_ctrl, -500.0f, -4500.0f, 0.0f, max_vx);
        } else {
            target_vx = 0.0f;
        }
        target_vy = 0.0f;
        if (roll_ctrl > 500.0f) {
            target_vy = remap(roll_ctrl, 500.0, 4500.0f, 0.0f, max_vy);
        } else if (roll_ctrl < -500.0f) {
            target_vy = remap(roll_ctrl, -500.0f, -4500.0f, 0.0f, min_vy);
        } else {
            target_vy = 0.0f;
        }
        target_vz = 0.0f;
        if (thr_ctrl < 400.0f) {
            target_vz = remap(thr_ctrl, 0.0f, 400.0f, min_vz, 0.0f);
        } else if (thr_ctrl > 600.0f) {
            target_vz = remap(thr_ctrl, 600.0f, 1000.0f, 0.0f, max_vz);
        } else {
            target_vz = 0;
        }

        float target_yaw_vel = 0.0f;
        if (yaw_ctrl < -500.0f) {
            target_yaw_vel = remap(yaw_ctrl, -4500.0f, -500.0f, -0.5f, 0.0f);
        } else if (yaw_ctrl > 500.0f) {
            target_yaw_vel = remap(yaw_ctrl, 500.0f, 4500.0f, 0.0f, 0.5f);
        } else {
            target_yaw_vel = 0.0;
        }
        target_yaw_diff += target_yaw_vel * 0.0025;

        // get NN input
        float observation[OB_SPACE_SIZE];
        get_observation_vector(observation);
        float action[AC_SPACE_SIZE];
        pi_act(observation, action);

        // Compute the desired thrust.
        for (int i = 0;i < AC_SPACE_SIZE;i ++) {
            _thrust_rpyt_out_NN[i] = action[i];
        }

        for (int i = 0;i < AC_SPACE_SIZE;i ++) {
            _thrust_rpyt_out_NN[i] = clamp(_thrust_rpyt_out_NN[i], 0.0, FINAL_BIAS * 2.0);
        }

        // save info to copter
        _copter.rpy[0] = observation[0]; _copter.rpy[1] = observation[1]; _copter.rpy[2] = observation[2];
        _copter.vel_ned[0] = observation[3]; _copter.vel_ned[1] = observation[4]; _copter.vel_ned[2] = observation[5];
        _copter.omega[0] = observation[6]; _copter.omega[1] = observation[7]; _copter.omega[2] = observation[8];
        _copter.target_vx = target_vx; _copter.target_vy = target_vy; _copter.target_vz = target_vz;
        _copter.target_yaw_diff = target_yaw_diff;
        for (int i = 0;i < AC_SPACE_SIZE;i++) {
            _copter.desired_thrust[i] = _thrust_rpyt_out_NN[i];
        }
        _copter.thr_ctrl_in = (int16_t)thr_ctrl;
    }

    _copter.yaw_0 = yaw_0;
}

/*
  call vehicle supplied thrust compensation if set. This allows
  vehicle code to compensate for vehicle specific motor arrangements
  such as tiltrotors or tiltwings
*/
void AP_MotorsHybrid::thrust_compensation(void)
{
    if (_thrust_compensation_callback) {
        _thrust_compensation_callback(_thrust_rpyt_out_NN, AP_MOTORS_MAX_NUM_MOTORS);
    }
}