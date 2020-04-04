#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
   
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->p_error = 0.0;
  this->i_error = 0.0;
  this->d_error = 0.0;

}

void PID::UpdateError(double cte) {
  
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

}

double PID::TotalError() {
    
  return (-Kp * p_error) - (Ki * i_error) - (Kd * d_error);

}

double PID::getKp() {
  return Kp;
}
double PID::getKi() {
  return Ki;
}
double PID::getKd() {
  return Kd;
}
double PID::get_p_error() {
  return p_error;
}
double PID::get_i_error() {
  return i_error;
}
double PID::get_d_error() {
  return d_error;
}