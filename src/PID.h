#ifndef PID_H
#define PID_H

#include <tuple>
#include <vector>

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
  std::tuple<double,double,double> GetPram(){
      return {kp_,kd_,ki_};
  }
      
  void Init(std::vector<double>&);

  double& GetTerr() {return i_error_;}
  double& GetSqerr() {return sq_cte_;}
  double GetAerr() {return sq_cte_/count_;} // vaerge sq error
  std::vector<double> GetCo() {return {kp_,kd_,ki_};} // vaerge sq error
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

 //private:
  /**
   * PID Errors
   */
  double total_error_ = 0;
  double p_error_ = 0;
  double d_error_ = 0;
  double i_error_= 0;
  double prev_cte= 0;
  double sq_cte_= 0;

  double count_ = 0; // record the loop update the cte error
  std::vector<double> errs_{999.0,999.0,999.0};

  /**
   * PID Coefficients
   */ 
  double kp_;
  double kd_;
  double ki_;
};

#endif  // PID_H
