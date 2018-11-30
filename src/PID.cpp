#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  PID::Kp = Kp;
  PID::Ki = Ki;
  PID::Kd = Kd;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  
  // Previous CTE
  prev_cte = 0.0;
}

void PID::UpdateError(double cte) {
  // Proportional error.
  p_error = cte;

  // Differential error.
  d_error = cte - prev_cte;
  prev_cte = cte;
  
  // Integral error.
  i_error += cte;
}

double PID::TotalError() {
  return p_error * Kp + i_error * Ki + d_error * Kd;
}

