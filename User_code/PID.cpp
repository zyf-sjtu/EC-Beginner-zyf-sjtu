//
// Created by zyfsj on 2026/10/11.
//
#include "PID.h"
PID::PID(float kp, float ki, float kd,float i_max,float out_max,float d_fliter_k):
kp_(kp),ki_(ki),kd_(kd),i_max_(i_max),out_max_(out_max),d_fliter_k_(d_fliter_k)
{
    err_=0;
    err_sum_=0;
    last_err_=0;
    last_dout_=0;
    ref_=0;
    fdb_=0;
}
void PID::reset()
{
    err_=0;
    err_sum_=0;
    last_err_=0;
    last_dout_=0;
    ref_=0;
    fdb_=0;
}
float PID::calc(float ref,float fdb)
{
    ref_=ref;
    fdb_=fdb;
    err_=ref-fdb;
    err_sum_=err_+err_sum_;
    pout_=kp_*err_;
    dout_=kd_*(err_-last_err_);
    iout_=ki_*err_sum_;
    if (iout_>i_max_) iout_=i_max_;
    if (iout_<-i_max_) iout_=-i_max_;
    dout_=(1-d_fliter_k_)*last_dout_+d_fliter_k_*dout_;
    last_err_=err_;
    last_dout_=dout_;
    if (pout_+dout_+iout_>out_max_) return out_max_;
    if (pout_+dout_+iout_<-out_max_) return -out_max_;
    return pout_+dout_+iout_;
}