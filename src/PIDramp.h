#ifndef PIDramp_H
#define PIDramp_H

// todo: implement feedforward
// todo: implement discretization
// todo: implement gain scheduled control


class PIDramp{
    public:
    /**
     *
     * @param kp - Proportional gain
     * @param ki - Integral gain
     * @param kd - Derivative gain
     * @param lower_limit - lower limit for control saturation
     * @param upper_limit - upper limit for control saturation
     * @param rate_limit - maximum rate of change of the ouput value
     */

        PIDramp(float kp, float ki, float kd, float lower_limit, float upper_limit, float rate_limit);

        float compute(float setpoint, float measured_value);
        void updateGains(float kp, float ki, float kd);
        void updateLimits(float lower_limit, float upper_limit, float rate_limit);
        void reset();

    private:
        float kp_;  // Proportional gain
        float ki_;  // Integral gain
        float kd_;  // Derivative gain
        float lower_limit_;  // minimum output value
        float upper_limit_;  // maximum output value
        float rate_limit_;  // maximum rate of change of the output value
        float integral_prev_;  // last integral component value
        float derivative_prev_;  // last derivative component value
        float error_prev_;  // last tracking error value
        float output_prev_;  // last pid output value
};

#endif