#ifndef HW_COMPONENTS_ALGORITHMS_PID_PID_HPP_
#define HW_COMPONENTS_ALGORITHMS_PID_PID_HPP_

namespace hello_world {
namespace pid {

class Pid {
 public:
    Pid() = default;
    Pid(float kp, float ki, float kd, float out_max, float integ_max);

    // 参数设置
    void setParams(float kp, float ki, float kd);
    void setLimit(float out_max, float integ_max);

    // 清空积分(状态/模式切换时)
    void reset();

    // 计算输出
    // vel_ref: 目标角速度（Kd·vel_ref 作为目标速度前馈）
    // vel_fb : 实测角速度（D 项用实测速度，不对位置做差分；yaw 传 IMU 陀螺 z）
    // integ_enable: false 时本拍不积分（到软件限位时用）
    float calc(float ref, float fdb, float vel_ref, float vel_fb,
               float dt = 0.001f, bool integ_enable = true);

    //数据访问
    float output() { return out_;}

 private:
    float kp_ = 0.0f;
    float ki_ = 0.0f;
    float kd_ = 0.0f;
    float out_max_ = 0.0f;
    float integ_max_ = 0.0f;

    float integ_ = 0.0f;     // 积分项
    float err_ = 0.0f;       // 本拍误差
    float out_ = 0.0f;       // 本拍输出
};

}}  // namespace

#endif  // HW_COMPONENTS_ALGORITHMS_PID_PID_HPP_