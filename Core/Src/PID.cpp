//
// Created by Administrator on 24-10-26.
//

#include "PID.h"

PID::PID(float kp, float ki, float kd, float i_max, float out_max)
    : kp_(kp), ki_(ki), kd_(kd), i_max_(i_max), out_max_(out_max),
      output_(0), ref_(0), fdb_(0), err_(0), err_sum_(0), last_err_(0),
      pout_(0), iout_(0), dout_(0) {}

float PID::calc(float ref, float fdb) {
    // update the err
    ref_ = ref;
    fdb_ = fdb;
    err_ = ref_ - fdb_;

    // calculate the pout_
    pout_ = kp_ * err_;

    // calculate the iout_
    err_sum_ += err_;
    if (err_sum_ > i_max_) err_sum_ = i_max_;
    if (err_sum_ < -i_max_) err_sum_ = -i_max_;
    iout_ = ki_ * err_sum_;

    // calculate the dout_
    dout_ = kd_ * (err_ - last_err_);

    // output the total
    output_ = pout_ + iout_ + dout_;
    if (output_ > out_max_) output_ = out_max_;
    if (output_ < -out_max_) output_ = -out_max_;

    // memorize the err
    last_err_ = err_;

    return output_;
}
