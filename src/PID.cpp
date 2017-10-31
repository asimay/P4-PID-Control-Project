#include "PID.h"
#include <cmath>
#include <limits>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
    p[0] = 0;
    p[1] = 0;
    p[2] = 0;

    dp[0] = dp[1] = 0.01;  //p,d
    dp[2] = 0.001; //i
	
	count = 0;
    is_initialized = false;
    debug = false;
}


PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    if(is_initialized == false) {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;

        p_error = 0.0;
        i_error = 0.0;
        d_error = 0.0;

        p[0] = Kp;
        p[1] = Kd;
        p[2] = Ki;

        best_err = std::numeric_limits<double>::max();
        err = 0.0;

        debug = true;
        cur_state = 0;
        last_index = 0;
        is_initialized = true;
    }
}

void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    i_error = i_error + cte;
    p_error = cte;

    return;
}

double PID::TotalError() {
    return 0;
}

double PID::GetSteerValue() {
    return -Kp * p_error - Kd * d_error - Ki * i_error;
}

void PID::TwiddleSetPIDValue() {
    Kp = p[0];
    Kd = p[1];
    Ki = p[2];

    return;
}

double PID::TotalTwiddleError() {
    return fabs(dp[0]) + fabs(dp[1]) + fabs(dp[2]);
}

void PID::twiddle(double cte) {
	count++;
    err = err + cte*cte;
	if(count < 500) return;
	
    switch(cur_state) {
    case 0 : {
        if(fabs(err) < fabs(best_err)) {
            best_err = err;
            p[last_index] += dp[last_index];
            dp[last_index] *= 1.1;
            cur_state = 3; //change index
        } else {
            p[last_index] -= 2 * dp[last_index];
            cur_state = 2;
        }
        break;
    }
    case 1: {
        p[last_index] += dp[last_index];
        cur_state = 0;
        break;
    }
    case 2: {
        if(fabs(err) < fabs(best_err)) {
            best_err = err;
            dp[last_index] *= 1.1;  //dp[0] = 1.1
        } else {
            p[last_index] += dp[last_index];
            dp[last_index] *= 0.9;
        }

        cur_state = 3;
        break;
    }
    case 3: {
        last_index = (last_index + 1) % 3;
        cur_state = 1;
        break;
    }
    }
	
	err = 0;
	count = 0.0;
	return;

}
