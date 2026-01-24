# Anti-Windup PID Controller (C++)

A small, dependency-free PID controller implementation with integrator anti-windup and a band-limited differentiator (first-order low-pass filter).

## Features
- Proportional, integral, and derivative terms with runtime tuning
- Anti-windup via integrator clamping against output limits
- Derivative on measurement with low-pass filtering (`tau`)
- Simple, embeddable `PID.h` / `PID.cpp`

## Usage
```cpp
#include "PID.h"

PID pid;

int main()
{
    // Gains and timing
    float Kp = 1.0f, Ki = 0.2f, Kd = 0.05f;
    float tau = 0.02f;  // derivative filter time constant
    float T = 0.01f;    // sample time

    pid.init(Kp, Ki, Kd, tau, T);
    pid.set_bounds(-100.0f, 100.0f);

    while (true)
    {
        float setpoint = 1.0f;
        float measurement = read_sensor();
        float output = pid.update(setpoint, measurement);
        write_actuator(output);
    }
}
```

## API
- `void init(float Kp, float Ki, float Kd, float tau, float T)`
- `float update(float setpoint, float measurement)`
- `void set_gains(float Kp, float Ki, float Kd, float tau, float T)`
- `void set_bounds(float min, float max)`
- `float get_p()`, `float get_i()`, `float get_d()`, `float get_tau()`, `float get_err()`

## Notes
- `T` should match the actual sample period of your control loop.
- Smaller `tau` yields a more responsive derivative; larger values smooth noise.
- Call `set_bounds` before `update` to enable output limiting and anti-windup.
