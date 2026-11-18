/**
 *******************************************************************************
 * @file      :gimbal.hpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

/* ============================================================================
 * 【新人任务】声明云台运动模块类 Gimbal（namespace robot）。
 *
 * 你要做：
 *   1. 设计 Gimbal 类，实现 yaw / pitch 两个关节的关节空间位置控制（位置环）。
 *   2. 读入云台目标角度（来自遥控器 / 键鼠 / 底盘经双板通讯传来的期望值），输出
 *      yaw / pitch 电机的控制量。
 *   3. 处理 yaw 轴 360° 连续旋转的角度回绕，pitch 轴用软件限位防堵转。
 *
 * 约束 / 提示：
 *   - yaw / pitch 均为达妙 DM-J4310 电机，反馈角度 / 指令接口见 inc/motor_DaMiao.hpp。
 *   - 位置环 PID 由你自己实现（PID 类），本模块只负责调用。
 * ==========================================================================*/
#ifndef GIMBAL_ROBOT_MODULES_GIMBAL_HPP_
#define GIMBAL_ROBOT_MODULES_GIMBAL_HPP_

#include "module_state.hpp"

namespace hello_world {
namespace motor {
class Motor;
}  // namespace motor
namespace pid {
class Pid;
}  // namespace pid
}  // namespace hello_world

namespace robot {

//   yaw  : 误差先归一化到 [-pi, pi) 再闭环
//   pitch: 软件限位 + 到限位停积分
class Gimbal {
 public:
    static constexpr float kTorqueLimit = 7.0f;           // 关节力矩上限(Nm)

    static constexpr float kPitchLowerLimit = -0.3f;
    static constexpr float kPitchUpperLimit = 0.58f;

    static constexpr float kYawTargetRateLimit = 8.0f;
    static constexpr float kPitchTargetRateLimit = 4.0f;   // 关节目标角速率上限(rad/s)

    static constexpr float kPitchFrictionComp = 0.25f;     // 补偿力矩(N·m)，置 0 即关闭
    static constexpr float kPitchFrictionBand = 0.002f;    // 完全退出点(rad)
    static constexpr float kPitchFrictionFull = 0.012f;    // 满补偿点(rad)，两点之间线性过渡

    static constexpr float kPitchGravityA = 0.80f;   // N·m，水平位托举力矩
    static constexpr float kPitchGravityB = 0.00f;   // N·m，偏心相位项

    static constexpr float kYawSpinFfGain = 0.05f;       // N·m per (rad/s)

    Gimbal() = default;
    ~Gimbal() = default;

    // 注册
    void registerMotor(hello_world::motor::Motor* yaw,
                       hello_world::motor::Motor* pitch);
    void registerPid(hello_world::pid::Pid* yaw, 
                     hello_world::pid::Pid* pitch);

    // 积分得目标角
    void addTargetDelta(const GimbalMotion& delta, float dt);
    // 重新使能 / 状态切换时让目标角跟随当前关节角，防止突变
    void syncTargetToActual();
    // 输出零力矩并复位 PID，同时让目标角跟随当前关节角
    void stop();

    // 周期任务
    // allow_output = false 时等价于 stop()
    // yaw_fb      : IMU 世界系 yaw 角 (rad)
    // yaw_rate_fb : IMU 陀螺 z 轴角速度 (rad/s)，作为 yaw 环 D 项反馈
    void update(float dt, bool allow_output, float yaw_fb, float yaw_rate_fb);

    // 底盘自转速率指令（小陀螺模式，rad/s）：给 yaw 轴做反向拖动前馈，其余模式传 0
    void setChassisSpinRate(float w) { spin_rate_ff_ = w; }

    // 自检保持：pitch 闭环到水平、yaw 不出力（IMU 未收敛时 yaw 环不可靠）。
    // 用于 Resurrection，防止云台失电下垂到机械限位，进 Working 时已经在水平位。
    void levelPitch(float dt, float yaw_fb);

    // 数据访问
    const GimbalMotion& target() const { return target_; }
    const GimbalMotion& actual() const { return actual_; }
    float yaw_torque() const { return yaw_torque_; }
    float pitch_torque() const { return pitch_torque_; }
    bool pitch_limited() const { return pitch_limited_; }

 private:
    // pitch 库仑摩擦补偿力矩(N·m)
    float pitchFrictionComp(float err) const;

    // pitch 重力补偿力矩(N·m)
    float pitchGravityTorque(float pitch) const;

    hello_world::motor::Motor* motor_yaw_ = nullptr;
    hello_world::motor::Motor* motor_pitch_ = nullptr;
    hello_world::pid::Pid* pid_yaw_ = nullptr;
    hello_world::pid::Pid* pid_pitch_ = nullptr;

    GimbalMotion target_;        // 关节目标角 (rad)
    GimbalMotion actual_;        // 关节反馈角 (rad)
    float yaw_torque_ = 0.0f;    // 本周期 yaw 力矩指令 (Nm)
    float pitch_torque_ = 0.0f;  // 本周期 pitch 力矩指令 (Nm)
    float yaw_rate_ref_ = 0.0f;  // 目标 yaw 角速度（目标角差分，rad/s）
    float yaw_rate_fb_ = 0.0f;   // 反馈 yaw 角速度（IMU 陀螺 z，rad/s）
    float spin_rate_ff_ = 0.0f;  // 底盘实测角速度（G8 前馈输入，rad/s）
    float last_target_yaw_ = 0.0f;  // 上一拍目标 yaw，用于算目标角速度
    bool pitch_limited_ = false; // 目标角是否已被 pitch 软件限位夹住
};

}  // namespace robot


#endif /* GIMBAL_ROBOT_MODULES_GIMBAL_HPP_ */