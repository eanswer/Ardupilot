// July 25, 2018
// Jie Xu

#include <AP_HAL/AP_HAL.h>
#include "AP_MotorsQuadPlane.h"
#include "../../ArduCopter/Copter.h"
#include "AP_QuadPlaneMatrices.h"

extern const AP_HAL::HAL& hal;

void copy_array(float* dst, float* src, uint16_t N) {
    for (uint16_t i = 0;i < N;++i)
        dst[i] = src[i];
}

void copy_array(uint32_t* dst, uint32_t* src, uint16_t N) {
    for (uint16_t i = 0;i < N;++i)
        dst[i] = src[i];
}

float clamp(float x, float l, float r) {
    if (x < l) return l;
    if (x > r) return r;
    return x;
}

float remap(float x, float l1, float r1, float l2, float r2) {
    // check if l1 != r1
    if (r1 - l1 < 1e-5)
        return l2;

    float clamped_x = clamp(x, l1, r1);

    return (x - l1) / (r1 - l1) * (r2 - l2) + l2;
}

float degree2radian(float degree) {
    return degree / 180.0f * PI;
}

float warp_radian(float radian) {
    for (;radian > PI;) radian -= PI * 2.0f;
    for (;radian < -PI;) radian += PI * 2.0f;
    return radian;
}

void stateDiff(float state1[], float state2[], float diff[]) {
    for (uint8_t i = 0;i < NUM_STATES;++i)
        diff[i] = state1[i] - state2[i];
    for (uint8_t i = 3;i < 6;++i)
        diff[i] = warp_radian(diff[i]);
}

AP_TrimStateController::AP_TrimStateController(float _K[][NUM_STATES], 
    float _state0[], float _u0[]) {
    
    copy_array(K[0], _K[0], NUM_ROTORS * NUM_STATES);
    copy_array(state0, _state0, NUM_STATES);
    copy_array(u0, _u0, NUM_ROTORS);
}

void AP_TrimStateController::set_K(float _K[][NUM_STATES]) {
    copy_array(K[0], _K[0], NUM_ROTORS * NUM_STATES);
}

void AP_TrimStateController::set_state0(float _state0[]) {
    copy_array(state0, _state0, NUM_STATES);
}

void AP_TrimStateController::set_u0(float _u0[]) {
    copy_array(u0, _u0, NUM_ROTORS);
}

AP_TransitionController::AP_TransitionController(float _K[][NUM_ROTORS][NUM_STATES],
    float _state0[][NUM_STATES], float _u0[][NUM_ROTORS], uint32_t _timestamp_ms[], 
    uint16_t _num_steps) {

    if (_num_steps < 1) {
        // something went wrong
        return;
    }

    num_steps = _num_steps;
    copy_array(K[0][0], _K[0][0], _num_steps * NUM_ROTORS * NUM_STATES);
    copy_array(state0[0], _state0[0], _num_steps * NUM_STATES);
    copy_array(u0[0], _u0[0], _num_steps * NUM_ROTORS);
    copy_array(timestamp_ms, _timestamp_ms, _num_steps);
    total_transition_time = _timestamp_ms[_num_steps - 1];
}

void AP_TransitionController::set_K(float _K[][NUM_ROTORS][NUM_STATES], uint16_t _num_steps) {

    if (_num_steps < 1 || (num_steps != 0 && _num_steps != num_steps)) {
        // something went wrong
        return;
    }

    num_steps = _num_steps;
    copy_array(K[0][0], _K[0][0], _num_steps * NUM_ROTORS * NUM_STATES);
}

void AP_TransitionController::set_state0(float _state0[][NUM_STATES], uint16_t _num_steps) {

    if (_num_steps < 1 || (num_steps != 0 && _num_steps != num_steps)) {
        // something went wrong
        return;
    }

    num_steps = _num_steps;
    copy_array(state0[0], _state0[0], _num_steps * NUM_STATES);
}

void AP_TransitionController::set_u0(float _u0[][NUM_ROTORS], uint16_t _num_steps) {

    if (_num_steps < 1 || (num_steps != 0 && _num_steps != num_steps)) {
        // something went wrong
        return;
    }

    num_steps = _num_steps;
    copy_array(u0[0], _u0[0], _num_steps * NUM_ROTORS);
}

void AP_TransitionController::set_timestamps(uint32_t _timestamp_ms[], uint16_t _num_steps) {

    if (_num_steps < 1 || (num_steps != 0 && _num_steps != num_steps)) {
        // something went wrong
        return;
    }

    num_steps = _num_steps;
    copy_array(timestamp_ms, _timestamp_ms, _num_steps);
}

void AP_MotorsQuadPlane::init(motor_frame_class frame_class, motor_frame_type frame_type) {

    current_mode = 0;

    // setup the motors
    setup_motors(frame_class, frame_type);

    // setup the controllers
    setup_controllers();

    // enable fast channels or instant pwm
    set_update_rate(_speed_hz);
}

void AP_MotorsQuadPlane::setup_motors(motor_frame_class frame_class, motor_frame_type frame_type) {
    // remove existing motors
    for (int8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        remove_motor(i);
    }

    if (frame_class == MOTOR_FRAME_QUADPLANE_CFG) {
        add_motor(AP_MOTORS_MOT_1, 0, 0, 1);
        add_motor(AP_MOTORS_MOT_2, 0, 0, 2);
        add_motor(AP_MOTORS_MOT_3, 0, 0, 3);
        add_motor(AP_MOTORS_MOT_4, 0, 0, 4);
        add_motor(AP_MOTORS_MOT_5, 0, 0, 5);
        _flags.initialised_ok = true;
    } else {
        _flags.initialised_ok = false;
    }
}

void AP_MotorsQuadPlane::setup_controllers() {

    // controller_copter.set_K(COPTER_K);
    // controller_copter.set_state0(COPTER_STATE0);
    // controller_copter.set_u0(COPTER_U0);
    
    // controller_gliding.set_K(GLIDING_K);
    // controller_gliding.set_state0(GLIDING_STATE0);
    // controller_gliding.set_u0(GLIDING_U0);

    // controller_copter_to_gliding.set_K(COPTER_TO_GLIDING_K, NUM_STEPS_COPTER_TO_GLIDING);
    // controller_copter_to_gliding.set_state0(COPTER_TO_GLIDING_STATE0, NUM_STEPS_COPTER_TO_GLIDING);
    // controller_copter_to_gliding.set_u0(COPTER_TO_GLIDING_U0, NUM_STEPS_COPTER_TO_GLIDING);
    // controller_copter_to_gliding.set_timestamps(COPTER_TO_GLIDING_TIMESTEPS, NUM_STEPS_COPTER_TO_GLIDING);

    // controller_gliding_to_copter.set_K(GLIDING_TO_COPTER_K, NUM_STEPS_GLIDING_TO_COPTER);
    // controller_gliding_to_copter.set_state0(GLIDING_TO_COPTER_STATE0, NUM_STEPS_GLIDING_TO_COPTER);
    // controller_gliding_to_copter.set_u0(GLIDING_TO_COPTER_U0, NUM_STEPS_GLIDING_TO_COPTER);
    // controller_gliding_to_copter.set_timestamps(GLIDING_TO_COPTER_TIMESTEPS, NUM_STEPS_GLIDING_TO_COPTER);
}

void AP_MotorsQuadPlane::output_to_motors() {

    int8_t i;
    int16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];    // final pwm values sent to the motor

    switch (_spool_mode) {
        case SHUT_DOWN: {
            // sends minimum values out to the motors
            // set motor output based on thrust requests
            for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    if (_disarm_disable_pwm && _disarm_safety_timer == 0 && !armed()) {
                        motor_out[i] = 0;
                    } else {
                        motor_out[i] = get_pwm_output_min();
                    }
                }
            }
            break;
        }
        case SPIN_WHEN_ARMED:
            // sends output to motors when armed but not flying
            for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    motor_out[i] = calc_spin_up_to_pwm();
                }
            }
            break;
        case SPOOL_UP:
        case THROTTLE_UNLIMITED:
        case SPOOL_DOWN:
            // set motor output based on thrust requests
            for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    // TODO: modify
                    motor_out[i] = calc_thrust_to_pwm(_thrust_rpyt_out[i]);
                }
            }
            break;
    }

    // send output to each motor
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            rc_write(i, motor_out[i]);
        }
    }
}

void AP_MotorsQuadPlane::output_armed_stabilizing() {

    float _roll_norm_in = _roll_radio_passthrough;
    float _pitch_norm_in = _pitch_radio_passthrough;
    float _yaw_norm_in = _yaw_radio_passthrough;
    float _throttle_norm_in = _throttle_radio_passthrough;
    
    update_mode();
    
    float state[NUM_STATES];
    float state0[NUM_STATES];
    float K[NUM_ROTORS][NUM_STATES];
    float u0[NUM_ROTORS];

    getStateSpaceVector(state);

    if (in_transition) {
        if (transition_direction == TRANSITION_COPTER_TO_GLIDING) {

        } else if (transition_direction == TRANSITION_GLIDING_TO_COPTER) {

        } else {
            // something went wrong
        }
    } else if (current_mode == QUADPLANE_COPTER_MODE) {
        // get state0
        for (uint8_t i = 0;i < NUM_STATES;++i)
            state0[i] = controller_copter.state0[i];
        
        // get current state vector
        float desired_altitude = remap(_throttle_norm_in, 0.0f, 1.0f, MIN_COPTER_ALTITUDE, MAX_COPTER_ALTITUDE);
        float desired_roll = remap(_roll_norm_in, -1.0f, 1.0f, degree2radian(MIN_ROLL_PITCH_DEGREE), degree2radian(MAX_ROLL_PITCH_DEGREE));
        float desired_pitch = remap(_pitch_norm_in, 1.0f, -1.0f, degree2radian(MIN_ROLL_PITCH_DEGREE), degree2radian(MAX_ROLL_PITCH_DEGREE));
        float desired_yaw_rate = remap(_throttle_norm_in, -1.0f, 1.0f, degree2radian(MIN_YAW_RATE_DEGREE), degree2radian(MAX_YAW_RATE_DEGREE));

        state0[2] = -desired_altitude;
        state0[3] = desired_roll;
        state0[4] = desired_pitch;
        state0[11] = desired_yaw_rate;

        state[0] = state0[0]; state[1] = state0[1];
        state[5] = state0[5];
        state0[2] = clamp(state0[2], state[2] - 1, state[2] + 1);

        // get K
        for (uint8_t i = 0;i < NUM_ROTORS;++i)
            for (uint8_t j = 0;j < NUM_STATES;++j)
                K[i][j] = controller_copter.K[i][j];
        
        // get u0
        for (uint8_t i = 0;i < NUM_ROTORS;++i)
            u0[i] = controller_copter.u0[i];
    } else if (current_mode == QUADPLANE_GLIDING_MODE) {

    } else {
        // something went wrong
    }

    // compute _thrust_rpyt_out by LQR
    float state_diff[NUM_STATES];
    stateDiff(state, state0, state_diff);
    
    float K_times_diff[NUM_ROTORS];
    for (uint8_t i = 0;i < NUM_ROTORS;++i) {
        K_times_diff[i] = 0;
        for (uint8_t j = 0;j < NUM_STATES;++j)
            K_times_diff[i] += K[i][j] * state_diff[j];
    }

    for (uint8_t i = 0;i < NUM_ROTORS;++i) {
        _thrust_rpyt_out[i] = -K_times_diff[i] + u0[i];
    }
}

void AP_MotorsQuadPlane::update_mode() {

    uint8_t mode_switch = _switch_CH6_passthrough;
    uint8_t input_mode;

    switch (mode_switch) {
        case 0:
            input_mode = QUADPLANE_COPTER_MODE;
            break;
        case 1:
            input_mode = QUADPLANE_GLIDING_MODE;
            break;
        default:
            return;
    }

    if (mode_switch != current_mode) {
        in_transition = true;
        current_mode = mode_switch;
        transition_start_time_ms = AP_HAL::millis();
        if (current_mode == QUADPLANE_COPTER_MODE) {
            transition_direction = TRANSITION_GLIDING_TO_COPTER;
        } else {
            transition_direction = TRANSITION_COPTER_TO_GLIDING;
        }
    }
}

// [0, 0, z, roll, pitch, yaw, vx_b, vy_b, vz_b, v_roll, v_pitch, v_yaw]
void AP_MotorsQuadPlane::getStateSpaceVector(float state[]) {

}