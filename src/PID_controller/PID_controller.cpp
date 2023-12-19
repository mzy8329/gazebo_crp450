#include "PID_controller.h"

void PIDController::calPIDOutput()
{    
    float err_now = ref_ - fdb_;

    output_ += p_*(err_now - err_[1]) + i_*err_now + d_*(err_now - 2*err_[0] + err_[1]);
    if(output_ > max_output_){ output_ = max_output_;}
    if(output_ < min_output_){ output_ = min_output_;}

    // ROS_INFO("%f___%f___%f___%f", output_, p_*(err_now - err_[1]), i_*err_now, d_*(err_now - 2*err_[0] + err_[1]));

    err_[1] = err_[0];
    err_[0] = err_now;
}