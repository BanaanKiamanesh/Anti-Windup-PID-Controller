#include "PID.h"

void PID::init(float Kp, float Ki, float Kd, float tau, float T) // Constructor
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->tau = tau;
    this->T = T;

    // Init to Zero
    integ = 0.0f;
    diff = 0.0f;
    prev_err = 0.0f;
    prev_meas = 0.0f;
    val = 0.0f;
}

float PID::update(float setpoint, float meas) // Calculate PID
{
    // Error Signal
    err = setpoint - meas;

    // Proportional Term
    prop = Kp * err;

    // Integral Term
    integ += 0.5f * Ki * T * (err + prev_err);

    // Anti-wind-up via Dynamic Integrator Clamping
    // Limits Computationa and Application
    if (lim_max > prop)
        lim_max_integ = lim_max - prop;
    else
        lim_max_integ = 0.0f;

    if (lim_min < prop)
        lim_max_integ = lim_min - prop;
    else
        lim_min_integ = 0.0f;

    // Constrain Integrator
    if (integ > lim_max_integ)
        integ = lim_max_integ;

    else if (integ < lim_min_integ)
        integ = lim_min_integ;

    // Derivative (Band - Limited Differentiator)
    /**** Derivative on Measurements ****/
    diff = (2.0f * Kd * (meas - prev_meas) + (2.0f * tau - T) * diff) / (2.0f * tau + T);

    // Calculate Output and Apply Limits
    val = prop + integ + diff;

    // Constrain Value in Given Bounds
    val = constrain(val, lim_min, lim_max);

    // Store error and Measurement for Later Use
    prev_err = err;
    prev_meas = meas;

    return val;
}

void PID::set_bounds(float min, float max)
{
    this->lim_max = max;
    this->lim_min = min;
}

void PID::set_gains(float Kp, float Ki, float Kd, float tau, float T)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->tau = tau;
    this->T = T;
}

float PID::get_p()
{
    return Kp;
}

float PID::get_i()
{
    return Ki;
}

float PID::get_d()
{
    return Kd;
}

float PID::get_tau()
{
    return tau;
}

float PID::get_err()
{
    return err;
}

float PID::constrain(float val, float low_bound, float up_bound)
{
    if (val < low_bound)
        val = low_bound;
    else if (val > up_bound)
        val = up_bound;

    return val;
}