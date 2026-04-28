#pragma once

class PIDController {
    public:
        PIDController(float kp, float ki, float kd, float max_output, float max_integral); //constructor
        float compute(float setpoint, float measured_value, float dt); //compute pid output
        void PIDreset();
        void setTuning(float kp, float ki, float kd);

        float getkp();
        float getki();
        float getkd();
        float getLastOutput() const;

    private:
        //tuning params
        float kp;
        float ki;
        float kd;
        float previous_error;
        float integral;
        float max_output;
        float max_integral;
        float last_output;
};
