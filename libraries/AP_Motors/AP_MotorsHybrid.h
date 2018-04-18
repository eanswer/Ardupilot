// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file   AP_MotorsHybrid.h
/// @brief  Motor control class for frames used for HybridUAV

#ifndef __AP_MOTORS_HYBRID_H__
#define __AP_MOTORS_HYBRID_H__

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include "AP_MotorsMatrix.h"    // Parent Motors Matrix library

// Forward declaration.
class Copter;

/// @class      AP_MotorsHybrid
class AP_MotorsHybrid : public AP_MotorsMatrix {
public:
    /// Constructor
    AP_MotorsHybrid(uint16_t loop_rate, uint16_t speed_hz, Copter& cop)
        : AP_MotorsMatrix(loop_rate, speed_hz), _copter(cop) { }
    virtual ~AP_MotorsHybrid() {}

    // setup_motors - configures the motors for rotors.
    virtual void        setup_motors(motor_frame_class frame_class, motor_frame_type frame_type);
    void                output_to_motors();

protected:
    // output - sends commands to the motors
    void                output_armed_stabilizing();
    void                thrust_compensation(void) override;

private:
    // Points to the Copter class so that we can get all kinds of sensor's data.
    Copter&       _copter;

    bool initialization_finished;
    double initial_yaw_sum;
    float initial_yaw;
    int yaw_count;
};

#endif  // AP_MOTORSHybrid
