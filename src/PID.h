#ifndef PID_H
#define PID_H

class PID {
  public:
    /*
    * Errors
    */
    double p_error;
    double i_error;
    double d_error;

    /*
    * Coefficients
    */
    double Kp;
    double Ki;
    double Kd;

    //twiddle usage
    double p[3];
    double dp[3];

    bool is_initialized;
    bool debug;
    double best_err;
    double err;
	int count;

    int cur_state;
    int last_index;

    /*
    * Constructor
    */
    PID();

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID.
    */
    void Init(double Kp, double Ki, double Kd);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();

    void twiddle(double cte);

    void TwiddleSetPIDValue();

    double TotalTwiddleError();

    double GetSteerValue();
};

#endif /* PID_H */
