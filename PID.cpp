#include <PID.h>

void PID::init(double Kp, double Ki, double Kd, double tau) // Constructor
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->tau = tau;

    // Init to Zero
    integ = 0.0f;
    diff = 0.0f;
    prev_err = 0.0f;
    prev_meas = 0.0f;
    val = 0.0f;
}

void PID::init() // Constructor
{
    // Init to Zero
    integ = 0.0f;
    diff = 0.0f;
    prev_err = 0.0f;
    prev_meas = 0.0f;
    val = 0.0f;
}

double PID::update(double setpoint, double meas) // Calculate PID
{
    // Error Signal
    double err = setpoint - meas;

    // Proportional Term
    double prop = Kp * err;

    // Integral Term
    integ += 0.5f * Ki * T * (err + prev_err);

    // Anti-wind-up via Dynamic Integrator Clamping
    double lim_min_integ, lim_max_integ;

    // Compute Limits
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

    if (val > lim_max)
        val = lim_max;
    else if (val < lim_min)
        val = lim_min;

    // Store error and Measurement for Later Use
    prev_err = err;
    prev_meas = meas;

    return val;
}

void PID::set_gains(double Kp, double Ki, double Kd, double tau)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->tau = tau;
}

double PID::get_p()
{
    return Kp;
}

double PID::get_i()
{
    return Ki;
}

double PID::get_d()
{
    return Kd;
}

double PID::get_tau()
{
    return tau;
}