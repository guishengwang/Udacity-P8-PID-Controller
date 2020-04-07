#include "PID.h"
#include <iostream>

using namespace std;
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd,double dp0, double di, double dd) {

  this->p[0] = Kp;
  this->p[1] = Ki;
  this->p[2] = Kd;
  this->dp[0] = dp0;
  this->dp[1] = di;
  this->dp[2] = dd;
  this->p_error = 0.0;
  this->i_error = 0.0;
  this->d_error = 0.0;
  this->best_error=0.0;
  this->i_PID=0;
  this->it=0;
  this->i_start=0;

}

void PID::UpdateError(double cte) {

  if(i_start==0){
    p_error=cte;
    i_start=100;
  }
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

}

double PID::TotalError() {
    
  return (-p[0] * p_error) - (p[1] * i_error) - (p[2] * d_error);

}

int PID::twiddle(int tw ,double total_cte){
    
    //first iteration
    if (it==0) {
      best_error=total_cte;
      p[i_PID]+=dp[i_PID];
      it+=1;
      cout<<"it==0  return 2  best_error="<<best_error<<endl;
      return 2;
    }
    

    if (tw==1) {
        p[i_PID]+=dp[i_PID];
        it+=1;
        cout<<"tw==1 return 2 best_error="<<best_error<<endl;

        return 2;
    }
    
    if (tw==2) {
      if (total_cte < best_error) {
        cout<<"tw==2 return 1 total_cte="<<total_cte<<" best_error="<< best_error<<endl;
        best_error=total_cte;
        dp[i_PID]*=1.1;
        it+=1;
        i_PID=(i_PID+1) % 3;
        return 1;
      }
      else {
        cout<<"tw==2 return 3 total_cte="<<total_cte<<" best_error="<< best_error<<endl;
        p[i_PID]-=2*dp[i_PID];
        return 3;
      }
    }
    
    
    if (tw==3){
      if (total_cte < best_error) {
        cout<<"tw==3 return 1 total_cte="<<total_cte<<" best_error="<< best_error<<endl;
        best_error=total_cte;
        dp[i_PID]*=1.1;
        it+=1;
        i_PID=(i_PID+1) % 3;
        return 1;
      }
      else {
        cout<<" tw=3,return 1 total_cte="<<total_cte<<" best_error="<< best_error<<endl;
        p[i_PID]+=dp[i_PID];
        dp[i_PID]*=0.9;
        it+=1;
        i_PID=(i_PID+1) % 3;
        return 1;
      }
    }
   cout<<"tw is out of range tw="<<tw<<endl;
   return 100;
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
