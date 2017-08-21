#include "PID.h"

#include <numeric>


using namespace std::chrono;

PID::PID() {
  p_error = 0;
  i_error = 0;
  d_error = 0;
  first_update = true;
  time_last_error = high_resolution_clock::now();
  error_i_window = new boost::circular_buffer<double>(100);
}

PID::~PID() {
  delete error_i_window;
}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->i_error = 0;
}

void PID::UpdateError(double cte) {
  // Only set cte_old if this is the first time we get called.
  if (first_update) {
    p_error = cte;
    first_update = false;
  }

  high_resolution_clock::time_point current_time = high_resolution_clock::now();
  duration<double> dt = duration_cast<duration<double>>(current_time - time_last_error);
  time_last_error = current_time;


  // Compute the derivative of error in respect to time (we assume delta_t to be 1).
  d_error = (cte - p_error) / dt.count();

  // Add the error to the integral of the error.
  error_i_window->push_back(i_error);
  i_error = accumulate(error_i_window->begin(), error_i_window->end(), 0.);
  // Safe the proportional part of the error.
  p_error = cte;
}

double PID::TotalError() {
  return -Kp * p_error - Kd * d_error - Ki * i_error;
}

