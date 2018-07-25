//// @file AP_MotorsQuadPlane.h
//// @brief Motor Control class for quadplanes
// July 25, 2018
// Jie Xu
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include "AP_MotorsMatrix.h"

class Copter;

#define MAX_STEPS 100
#define NUM_ROTORS 5
#define NUM_STATES 12

/// @class      AP_TrimStateController
class AP_TrimStateController {
public:
    AP_TrimStateController() {}
    AP_TrimStateController(float _K[][NUM_STATES], float _state0[], float _u0[]);
    ~AP_TrimStateController() {}

    void set_K(float _K[][NUM_STATES]);
    void set_state0(float _state0[]);
    void set_u0(float _u0[]);

    float K[NUM_ROTORS][NUM_STATES];
    float state0[NUM_STATES];
    float u0[NUM_ROTORS];
};

/// @class      AP_TransitionController
class AP_TransitionController {
public:
    AP_TransitionController() { num_steps = 0; }
    AP_TransitionController(float _K[][NUM_ROTORS][NUM_STATES], float _state0[][NUM_STATES],
        float _u0[][NUM_ROTORS], uint32_t _timestamps_ms[], uint16_t _num_steps);
    ~AP_TransitionController() {}

    void set_K(float _K[][NUM_ROTORS][NUM_STATES], uint16_t _num_steps);
    void set_state0(float _state0[][NUM_STATES], uint16_t _num_steps);
    void set_u0(float _u0[][NUM_ROTORS], uint16_t _num_steps);
    void set_timestamps(uint32_t _timestamps_ms[], uint16_t _num_steps);

    float K[MAX_STEPS][NUM_ROTORS][NUM_STATES];
    float state0[MAX_STEPS][NUM_STATES];
    float u0[MAX_STEPS][NUM_ROTORS];
    uint32_t timestamp_ms[MAX_STEPS];
    uint16_t num_steps;
    uint32_t total_transition_time;
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
    }

    /// Constructor
    AP_MotorsQuadPlane(uint16_t loop_rate, uint16_t speed_hz, Copter& cop) :
        AP_MotorsMatrix(loop_rate, speed_hz) { _copter = cop; }

    // init
    void init(motor_frame_class frame_class, motor_frame_type frame_type) override;
    // configures the motors for the defined frame_class and frame_type
    void setup_motors(motor_frame_class frame_class, motor_frame_type frame_type) override;

    void setup_controllers();

    // output_to_motors - sends output to named servos
    void output_to_motors() override;

protected:
    // calculate motor outputs
    void output_armed_stabilizing() override;
    void thrust_compensation(void) override {}
    void update_mode();
    void getStateSpaceVector(float state);
    
private:
    Copter&     _copter;
    uint8_t     current_mode;
    bool        in_transition;
    uint8_t     transition_direction;
    uint32_t    transition_start_time_ms;
    AP_TrimStateController  controller_copter;
    AP_TrimStateController  controller_gliding;
    AP_TransitionController controller_copter_to_gliding;
    AP_TransitionController controller_gliding_to_copter;
};
