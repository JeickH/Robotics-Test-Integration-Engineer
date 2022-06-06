/*! @package pid_controller
    Code Information:
        Maintainer: Eng. Davidson Daniel Rojas Cediel
        Mail: davidson@kiwibot.com
        Kiwi Campus / AI & Robotics Team
*/

#include "motion_control/pid_controller.hpp"

PIDController::PIDController() {}

float PIDController::ThrottlePID(float ref_vx, float cur_vx, double dt)
{
    /********************************************
     * DEFINE YOUR AMAZING PID CONTROLLER
     * Find Documentation here:
     * https://www.elprocus.com/the-working-of-a-pid-controller/
     ********************************************/
    if(m_throttle_ctrl == false){
        return ref_vx;
    };
    if(ref_vx == 0){
        // Reset the integral error
        m_vx_int_error = 0;
        return 0;
    }else{
    float fb_ctrl_offset = 1;
    // integral error
    m_vx_int_error += (ref_vx - cur_vx) * dt ;
    // PID equation
    float out =  fb_ctrl_offset + m_kp_thr * (ref_vx - cur_vx) + m_ki_thr * m_vx_int_error + m_kd_thr * (((ref_vx - cur_vx) - m_prev_prop_error)/ dt );
    // update last proportional error
    m_vx_prop_ek1 = ref_vx - cur_vx;
    
    return out;
    };
    
    /********************************************
     * END CODE
     *  ********************************************/
}

float PIDController::SteeringPID(float ref_wz, float cur_wz, double dt)
{
    /********************************************
     * DEFINE YOUR AMAZING PID CONTROLLER
     * Find Documentation here:
     * https://www.elprocus.com/the-working-of-a-pid-controller/
     *
     * FeedForward:
     * https://zhuanlan.zhihu.com/p/382010500#:~:text=In%20many%20applications,dynamic%20models%20used.
     * "Combined FeedForward and Feedback Control"
     ********************************************/

    if(m_steering_ctrl == false){
        return ref_wz;
    };
    if(ref_wz == 0){
        // Reset the integral error
        m_wz_int_error = 0;
        return 0;
    }else{
    
    float ff_ctrl_offset = 1;
    // integral error
    m_wz_int_error += (ref_wz - cur_wz) * dt ;
    // PID equation
    float out = ff_ctrl_offset + m_kp_thr * (ref_wz - cur_wz) + m_ki_thr * m_vx_int_error + m_kd_thr * (((ref_wz - cur_wz) - m_prev_prop_error)/ dt ) + m_kff_str * ref_wz ;
    // update last proportional error
    m_wz_prop_ek1 = ref_wz - cur_wz;
    
    return out;
    };
    /********************************************
     * END CODE
     *  ********************************************/
}
