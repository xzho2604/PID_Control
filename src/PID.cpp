#include "PID.h"
#include <vector>
#include <iostream>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(std::vector<double>& v){
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
    kp_ = v[0];
    kd_ = v[1];
    ki_ = v[2];
}


void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
    p_error_ = cte; // proportion error
    d_error_ = cte - prev_cte; // differentical error
    prev_cte = cte;
    i_error_ += cte; // update intergral error

    sq_cte_ += cte*cte;
    ++count_;
    //std::cout << " inside cte: " << cte << " cte**2: " << cte*cte << std::endl;

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
    return kp_*p_error_ +kd_*d_error_ +ki_*i_error_;
}
