## **PID Control Project**

The goals / steps of this project are the following:

* In this project, I'll implement a PID controller in C++ to maneuver the vehicle around the track.

---
## Pid Process


#### 1. PID parameters tuning.

In the project, PID is tuned by manual. I designed twiddle program but it work not very well now. I'll fine the twiddle code afterwards.

The P is Proportional term, which is proportional to the cross track error or CTE.

The I is Integral term, which is the sum of all previous deviations from reference, it is used to remove system biases.

The D is Derivative term, which is used to reduce oscillations and make the vehicle run following the trajectory smoothly.

In project, I tuned the PID parameters for: [0.13, 0.001, 3.0]. and it seems works well.

#### 2. steering value calculation:

`Steer_Value = -Kp*p_error - Ki*i_error - Kd*d_error)`

## Work afterwards:

Refine the twiddle code to make the program automated calculate the PID parameters.