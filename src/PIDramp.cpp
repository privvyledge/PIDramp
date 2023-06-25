#include <algorithm>
#include <chrono>
#include "PIDramp.h"
//#include "Arduino.h"

PIDramp::PIDramp(float kp, float ki, float kd, float lower_limit, float upper_limit, float rate_limit):
    kp_(kp),
    kd_(kd),
    ki_(ki),
    lower_limit_(lower_limit),  // min output supply  [rads/s]
    upper_limit_(upper_limit),  // max output supply  [rads/s]
    rate_limit_(rate_limit), // output derivative limit
    error_prev_(0.0f),
    derivative_prev_(0.0f),
    integral_prev_(0.0f)
    {
    }

float PIDramp::compute(float setpoint, float measured_value)
{
    float error;
    float integral;
    float derivative;
    float output;
    float output_saturated;

    //setpoint is constrained between min and max to prevent output from having too much error
    error = setpoint - measured_value;
    integral += error;
    derivative = error - error_prev_;

    if (setpoint == 0 && error == 0)
    {
        integral = 0;
    }

//    if (rate_limit_ > 0) {
//        // limit the rate of change
//        float output_rate = output - output_prev_;
//        if (output_rate > rate_limit_) {
//            output = output_prev_ + rate_limit_ * time_step;
//        }
//        else if (output_rate < -rate_limit_) {
//            output = output_prev_ - rate_limit_ * time_step;
//        }
//    }

    output = (kp_ * error) + (ki_ * integral) + (kd_ * derivative);

    // update previous values
    error_prev_ = error;
    derivative_prev_ = derivative;
    integral_prev_ = integral;

    // contrain values
//    output_saturated = constrain(output, lower_limit_, upper_limit_);
    output_saturated = std::max(lower_limit_, std::min(output, upper_limit_));
//    output_saturated = std::clamp(output, lower_limit_, upper_limit_);

    output_prev_ = output;
    return output_saturated;
}

void PIDramp::updateGains(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDramp::updateLimits(float lower_limit, float upper_limit, float rate_limit)
{
    lower_limit_ = lower_limit;
    upper_limit_ = upper_limit;
    rate_limit_ = rate_limit;
}

void PIDramp::reset()
{
    integral_prev_ = 0.0f;
    output_prev_ = 0.0f;
    error_prev_ = 0.0f;
}