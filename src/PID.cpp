#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd,double dp, double di, double dd) {

  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->dp = dp;
  this->di = di;
  this->dd = dd;
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

void PID::twiddle(){
    
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

double PID::getdp() {
  return dp;
}
double PID::getdi() {
  return di;
}
double PID::getdd() {
  return dd;
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
