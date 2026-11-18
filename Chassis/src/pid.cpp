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
    last_fdb_ = 0.0f;
    err_ = 0.0f;
    out_ = 0.0f;
    is_first_ = true;
}

float Pid::calc(float ref, float fdb, float dt){
    if (dt <= 0.0f) { dt = 0.001f; }   // 防止 D 项除零
    err_ = ref - fdb;

    // 微分先行（首拍不微分）
    float d = 0.0f;
    if (is_first_) {
        last_fdb_ = fdb;
        is_first_ = false;
    } else {
        d = -kd_ * (fdb - last_fdb_) / dt;
    }
    last_fdb_ = fdb;

    const float out_unsat = kp_ * err_ + integ_ + d;
    const bool saturating = (out_unsat > out_max_ && err_ > 0.0f) ||
                            (out_unsat < -out_max_ && err_ < 0.0f);
    if (!saturating) {
        integ_ += ki_ * err_ * dt;
        integ_ = Clamp(integ_, integ_max_);
    }

    out_ = Clamp(kp_ * err_ + integ_ + d, out_max_);
    return out_;
}

}}  //namespace 