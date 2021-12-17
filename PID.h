#ifndef _PID_H
#define _PID_H

class PID
{
private:
    float Kp, Ki, Kd;                             // Controller Gains
    float tau;                                    // Dericative low-pass Filter Time Const
    float T;                                      // Sampling Time
    float lim_min, lim_max;                       // Output Limits
    float integ, prev_err, diff, prev_meas, prop; // Controller Memory
    float val;                                    // PID output Value
    float err;                                    // Error (Setpoint - Measurement)
    float lim_min_integ, lim_max_integ;           // Anti Windup Integrator Limits
    float constrain(float, float, float);         // Function to Constrian a Value in Bounds

public:
    void init(float, float, float, float, float);
    float update(float, float);
    void set_gains(float, float, float, float, float);
    void set_bounds(float, float);
    float get_p();
    float get_i();
    float get_d();
    float get_tau();
    float get_err();
};
#endif