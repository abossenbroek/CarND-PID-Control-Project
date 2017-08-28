#ifndef PID_H
#define PID_H

#include <chrono>
#include <deque>

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

    bool first_update;
    std::chrono::high_resolution_clock::time_point time_last_error;
    std::deque<double>* error_i_window;

    unsigned long num_calls;
    long double total_error;
    unsigned int window_size_;

    /*
    * Constructor
    */
    PID(unsigned int window_size);

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

    /*
     * Get control value.
     */
    double GetControl();

    /*
     * Get average error.
     */
    double AverageError();

    /*
     * Get number of updates.
     */
    unsigned long GetNumUpdates();
};

#endif /* PID_H */
