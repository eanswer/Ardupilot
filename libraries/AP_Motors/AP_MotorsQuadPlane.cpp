// July 25, 2018
// Jie Xu

#include <AP_HAL/AP_HAL.h>
#include "AP_MotorsQuadPlane.h"
#include "../../ArduCopter/Copter.h"

extern const AP_HAL::HAL& hal;

void copy_array(float* dst, const float* src, uint16_t N) {
    for (uint16_t i = 0;i < N;++i)
        dst[i] = src[i];
}

void copy_array(uint32_t* dst, const uint32_t* src, uint16_t N) {
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

float wrap_radian(float radian) {

    if (radian > PI) radian -= PI * 2.0f;
    if (radian < -PI) radian += PI * 2.0f;

    return radian;
}

void stateDiff(float state1[], float state2[], float diff[]) {
    for (uint8_t i = 0;i < NUM_STATES;++i)
        diff[i] = state1[i] - state2[i];
    for (uint8_t i = 3;i < 6;++i)
        diff[i] = wrap_radian(diff[i]);
}

AP_TrimStateController::AP_TrimStateController(const float _K[][NUM_STATES], 
    const float _state0[], const float _u0[]) {
    
    copy_array(K[0], _K[0], NUM_ROTORS * NUM_STATES);
    copy_array(state0, _state0, NUM_STATES);
    copy_array(u0, _u0, NUM_ROTORS);
}

void AP_TrimStateController::set_K(const float _K[][NUM_STATES]) {
    copy_array(K[0], _K[0], NUM_ROTORS * NUM_STATES);
}

void AP_TrimStateController::set_state0(const float _state0[]) {
    copy_array(state0, _state0, NUM_STATES);
}

void AP_TrimStateController::set_u0(const float _u0[]) {
    copy_array(u0, _u0, NUM_ROTORS);
}

AP_TransitionController::AP_TransitionController(const float _K[][NUM_ROTORS][NUM_STATES],
    const float _state0[][NUM_STATES], const float _u0[][NUM_ROTORS], uint32_t _timestamp_ms[], 
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

void AP_TransitionController::set_K(const float _K[][NUM_ROTORS][NUM_STATES], uint16_t _num_steps) {

    if (_num_steps < 1 || (num_steps != 0 && _num_steps != num_steps)) {
        // something went wrong
        return;
    }

    num_steps = _num_steps;
    copy_array(K[0][0], _K[0][0], _num_steps * NUM_ROTORS * NUM_STATES);
}

void AP_TransitionController::set_state0(const float _state0[][NUM_STATES], uint16_t _num_steps) {

    if (_num_steps < 1 || (num_steps != 0 && _num_steps != num_steps)) {
        // something went wrong
        return;
    }

    num_steps = _num_steps;
    copy_array(state0[0], _state0[0], _num_steps * NUM_STATES);
}

void AP_TransitionController::set_u0(const float _u0[][NUM_ROTORS], uint16_t _num_steps) {

    if (_num_steps < 1 || (num_steps != 0 && _num_steps != num_steps)) {
        // something went wrong
        return;
    }

    num_steps = _num_steps;
    copy_array(u0[0], _u0[0], _num_steps * NUM_ROTORS);
}

void AP_TransitionController::set_timestamps(const uint32_t _timestamp_ms[], uint16_t _num_steps) {

    if (_num_steps < 1 || (num_steps != 0 && _num_steps != num_steps)) {
        // something went wrong
        return;
    }

    num_steps = _num_steps;
    copy_array(timestamp_ms, _timestamp_ms, _num_steps);
}

void AP_TransitionController::set_initial_altitude(const float _initial_altitude) {
    initial_altitude = _initial_altitude;
    index = 0;
}

// return if still in transition 
bool AP_TransitionController::get_controller(uint32_t transition_time, float _K[][NUM_STATES], float _state0[], float _u0[]) {
    for (;index < num_steps - 1 && transition_time > (timestamp_ms[index] + timestamp_ms[index + 1]) / 2.0;) {
        ++ index;
    }
    for (uint16_t i = 0;i < NUM_ROTORS;++i)
        for (uint16_t j = 0;j < NUM_STATES;++j)
            _K[i][j] = K[index][i][j];
    for (uint16_t i = 0;i < NUM_STATES;++i)
        _state0[i] = state0[index][i];
    for (uint16_t i = 0;i < NUM_ROTORS;++i)
        _u0[i] = u0[index][i];

    _state0[0] += -initial_altitude;

    return (transition_time < timestamp_ms[num_steps - 1]);
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

    controller_copter.set_K(COPTER_K);
    controller_copter.set_state0(COPTER_STATE0);
    controller_copter.set_u0(COPTER_U0);
    controller_copter.set_desired_altitude(0);

    controller_gliding.set_K(GLIDING_K);
    controller_gliding.set_state0(GLIDING_STATE0);
    controller_gliding.set_u0(GLIDING_U0);
    controller_gliding.set_desired_altitude(0);

    controller_copter_to_gliding.set_K(COPTER_TO_GLIDING_K, NUM_STEPS_COPTER_TO_GLIDING);
    controller_copter_to_gliding.set_state0(COPTER_TO_GLIDING_STATE0, NUM_STEPS_COPTER_TO_GLIDING);
    controller_copter_to_gliding.set_u0(COPTER_TO_GLIDING_U0, NUM_STEPS_COPTER_TO_GLIDING);
    controller_copter_to_gliding.set_timestamps(COPTER_TO_GLIDING_TIMESTEPS, NUM_STEPS_COPTER_TO_GLIDING);

    controller_gliding_to_copter.set_K(GLIDING_TO_COPTER_K, NUM_STEPS_GLIDING_TO_COPTER);
    controller_gliding_to_copter.set_state0(GLIDING_TO_COPTER_STATE0, NUM_STEPS_GLIDING_TO_COPTER);
    controller_gliding_to_copter.set_u0(GLIDING_TO_COPTER_U0, NUM_STEPS_GLIDING_TO_COPTER);
    controller_gliding_to_copter.set_timestamps(GLIDING_TO_COPTER_TIMESTEPS, NUM_STEPS_GLIDING_TO_COPTER);
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
                    // motor_out[i] = calc_thrust_to_pwm(_thrust_rpyt_out[i]);
                    if (i == 0) {
                        // motor 1 is the front motor
                        motor_out[i] = thrust_to_pwm_mapping_front(_thrust_rpyt_out[i], battery_voltage);
                    } else {
                        // motor 2~5 is quad motors
                        motor_out[i] = thrust_to_pwm_mapping_quad(_thrust_rpyt_out[i], battery_voltage);
                    }
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

int16_t AP_MotorsQuadPlane::thrust_to_pwm_mapping_front(float desired_thrust, float voltage) {
    // Reject unreasonable data.
    if (desired_thrust <= 0.0f || voltage <= 7.5f || voltage >= 13.0f) return 1000.0f;
    // Output from matlab:
    /*
    Linear model Poly21:
    a(x,y) = p00 + p10*x + p01*y + p20*x^2 + p11*x*y
      where x is normalized by mean 1521 and std 229.8086
      and where y is normalized by mean 11.7657 and std 0.4164
    Coefficients (with 95% confidence bounds):
       p00 =        3.28  (3.26, 3.3)
       p10 =       2.958  (2.939, 2.976)
       p01 =      0.2289  (0.2103, 0.2475)
       p20 =      0.6207  (0.6005, 0.6408)
       p11 =      0.1894  (0.1713, 0.2074)
    */
    const float mean_throttle = 1521.0f;
    const float std_throttle = 229.8086f;
    const float mean_voltage = 11.7657f;
    const float std_voltage = 0.4164f;
    const float p00 = 3.28f;
    const float p10 = 2.958f;
    const float p01 = 0.2289f;
    const float p20 = 0.6207f;
    const float p11 = 0.1894f;
    const float y = (voltage - mean_voltage) / std_voltage;
    const float a = p20;
    const float b = p11 * y + p10;
    const float c = p01 * y + p00 - desired_thrust;
    const float delta = b * b - 4 * a * c;
    if (delta < 0.0f) return 1000.0f;
    const float x = (-b + sqrtf(delta)) / 2.0f / a;
    const float throttle = x * std_throttle + mean_throttle;
    return clamp(throttle, 1000.0f, MAX_PWM);
}

int16_t AP_MotorsQuadPlane::thrust_to_pwm_mapping_quad(float desired_thrust, float voltage) {
    // Reject unreasonable data.
    if (desired_thrust <= 0.0f || voltage <= 7.5f || voltage >= 13.0f) return 1000.0f;
    // Output from matlab:
    /*
    Linear model Poly21:
    a(x,y) = p00 + p10*x + p01*y + p20*x^2 + p11*x*y
      where x is normalized by mean 1525 and std 230.9135
      and where y is normalized by mean 11.6555 and std 0.3697
    Coefficients (with 95% confidence bounds):
       p00 =       2.365  (2.355, 2.376)
       p10 =       2.171  (2.163, 2.178)
       p01 =      0.1652  (0.1579, 0.1724)
       p20 =      0.4747  (0.4666, 0.4828)
       p11 =      0.1151  (0.1078, 0.1224)
    */
    const float mean_throttle = 1525.0f;
    const float std_throttle = 230.9135f;
    const float mean_voltage = 11.6555f;
    const float std_voltage = 0.3697f;
    const float p00 = 2.365f;
    const float p10 = 2.171f;
    const float p01 = 0.1652f;
    const float p20 = 0.4747f;
    const float p11 = 0.1151f;
    const float y = (voltage - mean_voltage) / std_voltage;
    const float a = p20;
    const float b = p11 * y + p10;
    const float c = p01 * y + p00 - desired_thrust;
    const float delta = b * b - 4 * a * c;
    if (delta < 0.0f) return 1000.0f;
    const float x = (-b + sqrtf(delta)) / 2.0f / a;
    const float throttle = x * std_throttle + mean_throttle;
    return clamp(throttle, 1000.0f, MAX_PWM);
}

void AP_MotorsQuadPlane::output_armed_stabilizing() {

    float _roll_norm_in = radio_roll_in;
    float _pitch_norm_in = radio_pitch_in;
    float _yaw_norm_in = radio_yaw_in;
    float _throttle_norm_in = radio_throttle_in;
    
    update_mode();
    
    float state[NUM_STATES];
    float state0[NUM_STATES];
    float K[NUM_ROTORS][NUM_STATES];
    float u0[NUM_ROTORS];

    getStateSpaceVector(state);

    if (in_transition) {
        uint32_t transition_time = AP_HAL::millis() - transition_start_time_ms;

        if (transition_direction == TRANSITION_COPTER_TO_GLIDING) {
            stage = 2;
            in_transition = controller_copter_to_gliding.get_controller(transition_time, K, state0, u0);
            if (!in_transition) {
                controller_gliding.set_desired_altitude(controller_copter.get_desired_altitude());
            }
        } else if (transition_direction == TRANSITION_GLIDING_TO_COPTER) {
            stage = 3;
            in_transition = controller_gliding_to_copter.get_controller(transition_time, K, state0, u0);
        } else {
            // something went wrong
        }
        state[0] = state0[0]; state[1] = state0[1];
        state[5] = state0[5];
    } else if (current_mode == QUADPLANE_COPTER_MODE) {
        stage = 0;
        // get state0
        for (uint8_t i = 0;i < NUM_STATES;++i)
            state0[i] = controller_copter.state0[i];
        
        // get current state vector
        float desired_altitude = remap(_throttle_norm_in, 0.0f, 1.0f, MIN_COPTER_ALTITUDE, MAX_COPTER_ALTITUDE);
        float desired_roll = remap(_roll_norm_in, -1.0f, 1.0f, degree2radian(MIN_ROLL_PITCH_DEGREE), degree2radian(MAX_ROLL_PITCH_DEGREE));
        float desired_pitch = remap(_pitch_norm_in, -1.0f, 1.0f, degree2radian(MIN_ROLL_PITCH_DEGREE), degree2radian(MAX_ROLL_PITCH_DEGREE));
        float desired_yaw_rate = remap(_yaw_norm_in, -1.0f, 1.0f, degree2radian(MIN_YAW_RATE_DEGREE), degree2radian(MAX_YAW_RATE_DEGREE));

        controller_copter.set_desired_altitude(desired_altitude);

        // TODO: More consideration on sign of pitch
        state0[2] = -desired_altitude;
        state0[3] += desired_roll;
        state0[4] += desired_pitch;
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
        stage = 1;
        // get state0
        for (uint8_t i = 0;i < NUM_STATES;++i)
            state0[i] = controller_gliding.state0[i];
        
        // get current state vector
        float desired_altitude_rate = remap(_pitch_norm_in, -1.0f, 1.0f, MIN_GLIDING_ALTITUDE_RATE, MAX_GLIDING_ALTITUDE_RATE);
        float desired_roll = remap(_roll_norm_in, -1.0f, 1.0f, degree2radian(MIN_ROLL_PITCH_DEGREE), degree2radian(MAX_ROLL_PITCH_DEGREE));

        controller_gliding.desired_altitude += GLIDING_ALTITUDE_RATE_COEF * desired_altitude_rate; 
        
        state0[2] = -controller_gliding.desired_altitude;
        state0[3] = desired_roll;
        state0[11] = GLIDING_ROLL_TO_YAW_MIXING * desired_roll;

        state[0] = state0[0]; state[1] = state0[1];
        state[5] = state0[5];

        // get K
        for (uint8_t i = 0;i < NUM_ROTORS;++i)
            for (uint8_t j = 0;j < NUM_STATES;++j)
                K[i][j] = controller_gliding.K[i][j];
        
        // get u0
        for (uint8_t i = 0;i < NUM_ROTORS;++i)
            u0[i] = controller_gliding.u0[i];
    } else {
        // something went wrong
    }
    // save info for logging
    for (uint8_t i = 0;i < 12;++i) {
        last_state[i] = state[i];
        last_state0[i] = state0[i];
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

    uint16_t mode_switch = _radio_switch_ch6;
    uint8_t input_mode;

    if (mode_switch < 1300) {
        input_mode = QUADPLANE_COPTER_MODE;
    } else {
        input_mode = QUADPLANE_GLIDING_MODE;
    }

    if (input_mode != current_mode) {
        in_transition = true;
        current_mode = input_mode;
        transition_start_time_ms = AP_HAL::millis();
        if (current_mode == QUADPLANE_COPTER_MODE) {
            transition_direction = TRANSITION_GLIDING_TO_COPTER;
            controller_gliding_to_copter.set_initial_altitude(get_altitude());
        } else {
            transition_direction = TRANSITION_COPTER_TO_GLIDING;
            controller_copter_to_gliding.set_initial_altitude(get_altitude());
        }
    }
}

float AP_MotorsQuadPlane::get_altitude() {
    return altitude;
}

Vector3f AP_MotorsQuadPlane::get_velocity_in_body_frame() {
    float v_x_b = ned_velocity.y * sin_yaw + ned_velocity.x * cos_yaw;
    const float x_velocity_threshold = 2;
    if (current_mode == QUADPLANE_COPTER_MODE || v_x_b < x_velocity_threshold) {
        // if speed is slow, use accelerometer's estimation
    } else {
        v_x_b = airspeed;
    }
    float v_y_b = ned_velocity.y * cos_yaw - ned_velocity.x * sin_yaw;
    Vector3f velocity_body(v_x_b, v_y_b, ned_velocity.z);
    return velocity_body;
}

// [0, 0, z, roll, pitch, yaw, vx_b, vy_b, vz_b, v_roll, v_pitch, v_yaw]
void AP_MotorsQuadPlane::getStateSpaceVector(float state[]) {
    state[0] = 0; state[1] = 0; state[2] = -get_altitude();
    state[3] = roll; state[4] = pitch; state[5] = yaw;
    Vector3f velocity_body = get_velocity_in_body_frame();
    state[6] = velocity_body.x; state[7] = velocity_body.y; state[8] = velocity_body.z;
    state[9] = roll_rate; state[10] = pitch_rate; state[11] = yaw_rate;
}

// Set functions
void AP_MotorsQuadPlane::set_radio_switch(uint16_t switch_CH5, uint16_t switch_CH6) {
    _radio_switch_ch5 = switch_CH5;
    _radio_switch_ch6 = switch_CH6;
}

void AP_MotorsQuadPlane::set_radio_rpyt(float radio_roll, float radio_pitch, float radio_throttle, float radio_yaw) {
    radio_roll_in = radio_roll; 
    radio_pitch_in = radio_pitch; 
    radio_throttle_in = radio_throttle;
    radio_yaw_in = radio_yaw;
}

void AP_MotorsQuadPlane::set_attitude(float _roll, float _pitch, float _yaw) {
    roll = _roll;
    pitch = _pitch;
    yaw = _yaw;
}

void AP_MotorsQuadPlane::set_trig(float _sin_roll, float _sin_pitch, float _sin_yaw, 
    float _cos_roll, float _cos_pitch, float _cos_yaw) {
    sin_roll = _sin_roll;
    sin_pitch = _sin_pitch;
    sin_yaw = _sin_yaw;
    cos_roll = _cos_roll;
    cos_pitch = _cos_pitch;
    cos_yaw = _cos_yaw;
}

void AP_MotorsQuadPlane::set_attitude_rate(float _roll_rate, float _pitch_rate, float _yaw_rate) {
    roll_rate = _roll_rate;
    pitch_rate = _pitch_rate;
    yaw_rate = _yaw_rate;
}

void AP_MotorsQuadPlane::set_altitude(float _altitude) {
    altitude = _altitude;
}

void AP_MotorsQuadPlane::set_ned_velocity(Vector3f _ned_velocity) {
    ned_velocity.x = _ned_velocity.x;
    ned_velocity.y = _ned_velocity.y;
    ned_velocity.z = _ned_velocity.z;
}

void AP_MotorsQuadPlane::set_airspeed(float _airspeed) {
    airspeed = _airspeed;
}

void AP_MotorsQuadPlane::set_battery_voltage(float _voltage) {
    battery_voltage = _voltage;
}

void AP_MotorsQuadPlane::get_state(float _state[]) {
    for (uint8_t i = 0;i < 12;++i) {
        _state[i] = last_state[i];
    }
}

void AP_MotorsQuadPlane::get_state0(float _state0[]) {
    for (uint8_t i = 0;i < 12;++i) {
        _state0[i] = last_state0[i];
    }
}