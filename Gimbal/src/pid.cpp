#include "pid.hpp"
 
namespace hello_world{
namespace pid{


//限幅
namespace{
float Clamp(float x, float lim){
    if(x > lim)  x = lim;
    else if(x < -lim)  x = -lim;
    return x;
}}

Pid::Pid(float kp, float ki, float kd, float out_max, float integ_max)
        : kp_(kp), ki_(ki), kd_(kd), out_max_(out_max), integ_max_(integ_max) {}

void Pid::setParams(float kp, float ki, float kd){
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void Pid::setLimit(float out_max, float integ_max){
    out_max_ = out_max;
    integ_max_ = integ_max;
}

void Pid::reset(){
    integ_ = 0.0f;
    err_ = 0.0f;
    out_ = 0.0f;
}

float Pid::calc(float ref, float fdb, float vel_ref, float vel_fb, float dt,
                bool integ_enable){
    if (dt <= 0.0f) { dt = 0.001f; }
    err_ = ref - fdb;

    const float d = kd_ * (vel_ref - vel_fb);  // 目标速度前馈 & 阻尼

    const float out_unsat = kp_ * err_ + integ_ + d;
    const bool saturating = (out_unsat > out_max_ && err_ > 0.0f) ||
                            (out_unsat < -out_max_ && err_ < 0.0f);  // 条件积分抗饱和
    if (!saturating && integ_enable) {
        integ_ += ki_ * err_ * dt;
        integ_ = Clamp(integ_, integ_max_);
    }

    out_ = Clamp(kp_ * err_ + integ_ + d, out_max_);
    return out_;
}

}}  //namespace 