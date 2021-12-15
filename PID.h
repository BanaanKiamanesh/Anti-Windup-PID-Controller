#ifndef _PID_H
#define _PID_H

class PID
{
private:
    double Kp, Ki, Kd;                       // Controller Gains
    double tau;                              // Dericative low-pass Filter Time Const
    double T;                                // Sampling Time
    double lim_min, lim_max;                 // Output Limits
    double integ, prev_err, diff, prev_meas; // Controller Memory
    double val;                              // PID output Value

public:
    void init(double, double, double, double);
    void init();
    double update(double, double);
    void set_gains(double, double, double, double);
    double get_p();
    double get_i();
    double get_d();
    double get_tau();
};
#endif