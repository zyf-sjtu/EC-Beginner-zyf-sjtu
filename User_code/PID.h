//
// Created by zyfsj on 2026/10/11.
//

#ifndef PID_PID_H
#define PID_PID_H

class PID
{
public:
    PID(float kp, float ki, float kd,float i_max,float out_max,float d_fliter_k);
   void reset();
    float calc(float ref,float fdb);
    float kp_;
    float ki_;
    float kd_;
    float i_max_;
    float out_max_;
    float d_fliter_k_;
    float ref_;
    float fdb_;
private:

    float err_;
    float err_sum_;
    float last_err_;
    float last_dout_;
    float pout_;
    float dout_;
    float iout_;
};

#endif //PID_PID_H
