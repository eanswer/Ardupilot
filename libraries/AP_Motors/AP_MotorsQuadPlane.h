//// @file AP_MotorsQuadPlane.h
//// @brief Motor Control class for quadplanes
// July 25, 2018
// Jie Xu
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include "AP_MotorsMatrix.h"
#include "AP_QuadPlaneMatrices.h"

class Copter;

#define MAX_STEPS 40
#define MIN_COPTER_ALTITUDE -3
#define MAX_COPTER_ALTITUDE 5
#define MIN_ROLL_PITCH_DEGREE -30
#define MAX_ROLL_PITCH_DEGREE 30
#define MIN_YAW_RATE_DEGREE -45
#define MAX_YAW_RATE_DEGREE 45
#define MIN_GLIDING_ALTITUDE_RATE -2
#define MAX_GLIDING_ALTITUDE_RATE 2
#define GLIDING_ROLL_TO_YAW_MIXING 1
#define GLIDING_ALTITUDE_RATE_COEF 0.0125
#define MAX_PWM 1900

#ifndef PI
    #define PI 3.1415927
#endif

/// @class      AP_TrimStateController
class AP_TrimStateController {
public:
    AP_TrimStateController() {}
    AP_TrimStateController(const float _K[][NUM_STATES], const float _state0[], const float _u0[]);
    ~AP_TrimStateController() {}

    void set_K(const float _K[][NUM_STATES]);
    void set_state0(const float _state0[]);
    void set_u0(const float _u0[]);
    void set_desired_altitude(const float _desired_altitude) { desired_altitude = _desired_altitude; }
    float get_desired_altitude() { return desired_altitude; }

    float K[NUM_ROTORS][NUM_STATES];
    float state0[NUM_STATES];
    float u0[NUM_ROTORS];
    float desired_altitude; // used for gliding controller
};

/// @class      AP_TransitionController
class AP_TransitionController {
public:
    AP_TransitionController() { num_steps = 0; }
    AP_TransitionController(const float _K[][NUM_ROTORS][NUM_STATES], const float _state0[][NUM_STATES],
        const float _u0[][NUM_ROTORS], uint32_t _timestamps_ms[], uint16_t _num_steps);
    ~AP_TransitionController() {}

    void set_K(const float _K[][NUM_ROTORS][NUM_STATES], uint16_t _num_steps);
    void set_state0(const float _state0[][NUM_STATES], uint16_t _num_steps);
    void set_u0(const float _u0[][NUM_ROTORS], uint16_t _num_steps);
    void set_timestamps(const uint32_t _timestamp_ms[], uint16_t _num_steps);
    void set_initial_altitude(const float _initial_altitude);
    bool get_controller(uint32_t transition_time, float _K[][NUM_STATES], float _state0[], float _u0[]);

    float K[MAX_STEPS][NUM_ROTORS][NUM_STATES];
    float state0[MAX_STEPS][NUM_STATES];
    float u0[MAX_STEPS][NUM_ROTORS];
    uint32_t timestamp_ms[MAX_STEPS];
    uint16_t num_steps;
    uint32_t total_transition_time;
    float initial_altitude;
    uint16_t index;
};

/// @class      AP_MotorsQuadPlane
class AP_MotorsQuadPlane : public AP_MotorsMatrix {
public:
    enum {
        QUADPLANE_COPTER_MODE = 0,
        QUADPLANE_GLIDING_MODE = 1,
    };

    enum {
        TRANSITION_COPTER_TO_GLIDING = 0,
        TRANSITION_GLIDING_TO_COPTER = 1,
    };

    /// Constructor
    AP_MotorsQuadPlane(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMatrix(loop_rate, speed_hz) {}

    // init
    void init(motor_frame_class frame_class, motor_frame_type frame_type) override;
    // configures the motors for the defined frame_class and frame_type
    void setup_motors(motor_frame_class frame_class, motor_frame_type frame_type) override;

    void setup_controllers();

    // output_to_motors - sends output to named servos
    void output_to_motors() override;

    void set_radio_switch(uint16_t switch_CH5, uint16_t switch_CH6) override;
    void set_radio_rpyt(float radio_roll, float radio_pitch, float radio_throttle, float radio_yaw) override;
    void set_attitude(float _roll, float _pitch, float _yaw) override;
    void set_trig(float _sin_roll, float _sin_pitch, float _sin_yaw, float _cos_roll, float _cos_pitch, float _cos_yaw) override;
    void set_attitude_rate(float _roll_rate, float _pitch_rate, float _yaw_rate) override;
    void set_altitude(float _altitude) override;
    void set_ned_velocity(Vector3f _ned_velocity) override;
    void set_airspeed(float _airspeed) override;
    void set_battery_voltage(float _voltage) override;

    uint8_t get_current_mode() { return current_mode; }
    bool    get_in_transition() { return in_transition; }
    float   get_radio_roll_in() { return radio_roll_in; }
    float   get_radio_pitch_in() { return radio_pitch_in; }
    float   get_radio_throttle_in() { return radio_throttle_in; }
    float   get_radio_yaw_in() { return radio_yaw_in; }
    float   get_airspeed() { return airspeed; }
    
protected:
    // thrust to pwm mappings
    int16_t thrust_to_pwm_mapping_front(float desired_thrust, float voltage);
    int16_t thrust_to_pwm_mapping_quad(float desired_thrust, float voltage);

    // calculate motor outputs
    void output_armed_stabilizing() override;
    void thrust_compensation(void) override {}
    void update_mode();
    float get_altitude();
    Vector3f get_velocity_in_body_frame();
    void getStateSpaceVector(float state[]);

private:
    // --------------------------------------------------------------------------------------
    // July 25, 2018
    // Jie Xu
    uint16_t _radio_switch_ch5 = 0;
    uint16_t _radio_switch_ch6 = 0;
    float radio_roll_in, radio_pitch_in, radio_throttle_in, radio_yaw_in;
    
    float roll, pitch, yaw;
    float sin_roll, sin_pitch, sin_yaw;
    float cos_roll, cos_pitch, cos_yaw;
    float roll_rate, pitch_rate, yaw_rate;
    float altitude;
    Vector3f ned_velocity;
    float airspeed;
    float battery_voltage;
    // --------------------------------------------------------------------------------------

    uint8_t     current_mode;
    bool        in_transition; 
    uint8_t     transition_direction;
    uint32_t    transition_start_time_ms;
    AP_TrimStateController  controller_copter;
    AP_TrimStateController  controller_gliding;
    AP_TransitionController controller_copter_to_gliding;
    AP_TransitionController controller_gliding_to_copter;
};
