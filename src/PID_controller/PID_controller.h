#ifndef _PID_CONTROLLER_H_
#define _PID_CONTROLLER_H_

#include <ros/ros.h>
#include <std_msgs/Float32.h>


class PIDController
{
public:
    PIDController()
    {
        err_[0] = 0, err_[1] = 0;
        output_ = 0;
        ref_ = 0;
    }
    PIDController(float p, float i, float d, float max_output = 100, float min_output = -100)
    :p_(p), i_(i), d_(d), max_output_(max_output), min_output_(min_output)
    {
        err_[0] = 0, err_[1] = 0;
        output_ = 0;
        ref_ = 0;
    }
    ~PIDController(){;}

    void loadParam(float p, float i, float d, float max_output = 100, float min_output = -100)
    {
        p_ = p;
        i_ = i;
        d_ = d;
        max_output_ = max_output;
        min_output_ = min_output;
    }
    void calPIDOutput();

    float output_;
    float ref_;
    float fdb_;

    float p_;
    float i_;
    float d_;
    float max_output_;
    float min_output_;

private:

    float err_[2];
};


#endif // _PID_CONTROLLER_H_
