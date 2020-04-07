#ifndef PID_H
#define PID_H

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp, double Ki, double Kd, double dp0, double di, double dd);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  int twiddle(int tw, double total_cte);

  double getKp();
  double getKi();
  double getKd();
  double getdp();
  double getdi();
  double getdd();
  double get_p_error();
  double get_i_error();
  double get_d_error();

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;
  double best_error;
  double p[3];
  double dp[3];
  int i_PID;
  int it;
  int i_start;
  /**
   * PID Coefficients
   */ 
  //double Kp;
  //double Ki;
  //double Kd;
  //double dp;
  //double di;
  //double dd;


};

#endif  // PID_H
