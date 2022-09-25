#pragma once
#include "../core/include/utils/pid.h"
#include "../core/include/utils/feedforward.h"
#include "../core/include/utils/trapezoid_profile.h"
#include "vex.h"

/**
 * Motion Controller class
 * 
 * This class defines a top-level motion profile, which can act as an intermediate between
 * a subsystem class and the motors themselves
 *
 * This takes the constants kS, kV, kA, kP, kI, kD, max_v and acceleration and wraps around 
 * a feedforward, PID and trapezoid profile. It does so with the following formula:
 * 
 * out = feedfoward.calculate(motion_profile.get(time_s)) + pid.get(motion_profile.get(time_s))
 * 
 * For PID and Feedforward specific formulae, see pid.h, feedforward.h, and trapezoid_profile.h
 * 
 * @author Ryan McGee
 * @date 7/13/2022
 */
class MotionController
{
    public:

    /**
     * @brief Construct a new Motion Controller object
     * 
     * @param max_v Maximum velocity the movement is capable of
     * @param accel Acceleration / deceleration of the movement
     * @param pid_cfg Definitions of kP, kI, and kD
     * @param ff_cfg Definitions of kS, kV, and kA
     */
    MotionController(double max_v, double accel, PID::pid_config_t &pid_cfg, FeedForward::ff_config_t &ff_cfg);

    /**
     * @brief Initialize the motion profile for a new movement
     * This will also reset the PID and profile timers.
     * 
     * @param start_pt Movement starting position
     * @param end_pt Movement ending posiiton 
     */
    void init(double start_pt, double end_pt);
    
    /**
     * @brief Update the motion profile with a new sensor value
     * 
     * @param sensor_val Value from the sensor
     * @return The motor input generated from the motion profile 
     */
    double update(double sensor_val);

    /** 
     * @return Whether or not the movement has finished, and the PID
     * confirms it is on target
     */
    bool is_on_target();

    private: 

    double max_v, accel;
    PID::pid_config_t &pid_cfg;

    PID pid;
    FeedForward ff;
    TrapezoidProfile profile;
     
    vex::timer tmr;

};