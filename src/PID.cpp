#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd,double dp, double di, double dd) {

  this->p[0] = Kp;
  this->p[1] = Ki;
  this->p[2] = Kd;
  this->dp[0] = dp;
  this->dp[1] = di;
  this->dp[2] = dd;
  this->p_error = 0.0;
  this->i_error = 0.0;
  this->d_error = 0.0;
  this->i_PID=0;

}

void PID::UpdateError(double cte) {

  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

}

double PID::TotalError() {
    
  return (-p[0] * p_error) - (p[1] * i_error) - (p[2] * d_error);

}

void PID::twiddle(double best_error,double total_cte){
  
    
    
}

double PID::getKp() {
  return p[0];
}
double PID::getKi() {
  return p[1];
}
double PID::getKd() {
  return p[2];
}

double PID::getdp() {
  return dp[0];
}
double PID::getdi() {
  return dp[1];
}
double PID::getdd() {
  return dp[2];
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


/*
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
  this->i_PID=0;

}

void PID::UpdateError(double cte) {

  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

}

double PID::TotalError() {
    
  return (-Kp * p_error) - (Ki * i_error) - (Kd * d_error);

}
*/
