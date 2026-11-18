/**
 *******************************************************************************
 * @file      :chassis.cpp
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
 * 【新人任务】实现 chassis.hpp 中声明的底盘运动学。
 *
 * 你要做：
 *   1. 逆解：输入 (vx, vy, w)，输出每个舵的目标转角与每个轮的轮速 / 电流指令。
 *   2. 舵向轮需处理舵角连续性：当目标舵角与当前舵角相差超过 π/2 时，选择等效的
 *      另一组解（舵角反向、轮速取反），避免舵机大角度回摆。
 *   3. 对解算结果做限幅，避免超速。
 *
 * 约束 / 提示：
 *   - 解算结果会经电机组件的 encode() 下发，注意输入量的类型（电流 / 转矩）。
 * ==========================================================================*/

#include "chassis.hpp"
#include <cmath>
#include "assert.hpp"
#include "base.hpp"
#include "motor_base.hpp"

// 参数
namespace {

// 轮组安装位置（车体系，x 前 / y 左）
// 0 左前 / 1 左后 / 2 右后 / 3 右前
constexpr float kWheelX[4] = {0.167f, -0.167f, -0.167f, 0.167f};
constexpr float kWheelY[4] = {0.167f, 0.167f, -0.167f, -0.167f};

constexpr float kIdleSteerAngle[4] = {3.0f * PI / 4.0f, -3.0f * PI / 4.0f,
                                      -PI / 4.0f, PI / 4.0f};

// 单个轮组的线速度上限 (m/s) 超出后整体等比缩放
constexpr float kWheelSpeedMax = 3.0f;

// 逆解死区：轮心目标线速度低于该值时不转舵 (m/s)
constexpr float kSolveSpeedDeadband = 0.02f;

// 舵角回中：中位容差（rad）与输出限幅（raw）
constexpr float kSteerCenterTol = 0.03f;
constexpr float kSteerCenterOutMax = 3000.0f;

// 舵电机为电流控制模式：raw 指令换算成安培（PID 满量程 9000 raw 对应 3 A）
constexpr float kSteerCurrPerRaw = 3.0f / 9000.0f;

// 舵角目标整形：一阶低通 + 速率限制
constexpr float kSteerFilterTau = 0.020f;  // 目标方向低通时间常数 (s)
constexpr float kSteerRateMax = 20.0f;     // 目标舵角速率上限 (rad/s)

// 舵角速度前馈：按参考角速度前馈电流，抵消舵组粘滞阻尼
constexpr float kSteerVelFfA = 0.03f;      // A per (rad/s)

// 小陀螺角速度前馈：把 δ 前推的时间常数 (s)
constexpr float kDeltaLeadTime = 0.005f;   

// 跟随模式 δ 角度环参数
constexpr float kFollowKp = 4.0f;
constexpr float kFollowKi = 0.5f;
constexpr float kFollowKd = 0.0f;         
constexpr float kFollowIntegMax = 2.0f;
constexpr float kFollowDeltaTau = 0.020f;  // δ 低通时间常数 (s)
constexpr float kFollowKdGyro = -0.60f;     // 阻尼系数，作用在底盘陀螺上（干净信号）

}  // namespace

robot::Chassis::Chassis(): pid_follow_(kFollowKp, kFollowKi, 
                   kFollowKd, kMaxFollowYawRate, kFollowIntegMax) {}

// motor/pid注册
void robot::Chassis::registerMotorWheel(int idx, hello_world::motor::Motor* motor){
    HW_ASSERT(0 <= idx && idx < 4, "Error wheel idx: %d", idx);
    motor_wheel_[idx] = motor;
}
void robot::Chassis::registerMotorSteer(int idx, hello_world::motor::Motor* motor){
    HW_ASSERT(0 <= idx && idx < 4, "Error steer idx: %d", idx);
    motor_steer_[idx] = motor;
}
void robot::Chassis::registerPidWheel(int idx, hello_world::pid::Pid* pid){
    HW_ASSERT(0 <= idx && idx < 4, "Error wheel pid idx: %d", idx);
    pid_wheel_[idx] = pid;
}
void robot::Chassis::registerPidSteer(int idx, hello_world::pid::Pid* pid){
    HW_ASSERT(0 <= idx && idx < 4, "Error steer pid idx: %d", idx);
    pid_steer_[idx] = pid;
}

// 读取模式，输入Motion，完成旋转，完成逆解，pid calc，电机setinput
void robot::Chassis::update(const ChassisMotion& motion, float w_manual, float dt){
    // 根据工作模式，得到底盘 w 
    // 底盘的 w 由工作模式决定：
    // 分离->0；跟随->由 δ 角环给出；小陀螺->w_manual
    yaw_rate_ = calcYawRate(w_manual, dt);

    // 旋转：云台系 -> 车系
    // 角速度前馈：把 δ 前推 kDeltaLeadTime
    const float delta_pred =hello_world::AngleNormRad(delta_ + kDeltaLeadTime * gyro_z_);
    const float cos_delta = cosf(delta_pred);
    const float sin_delta = sinf(delta_pred);
    const float vx = motion.vx * cos_delta - motion.vy * sin_delta;
    const float vy = motion.vx * sin_delta + motion.vy * cos_delta;

    // 四轮逆解 (初步分解)
    float vix[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float viy[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float speed_max = 0.0f;
    for (int i = 0; i < 4u; i++) {
        vix[i] = vx - yaw_rate_ * kWheelY[i];
        viy[i] = vy + yaw_rate_ * kWheelX[i];
        const float speed = sqrtf(vix[i] * vix[i] + viy[i] * viy[i]);
        if (speed > speed_max) {  speed_max = speed;  }
    }

    // 四轮限速（等比缩放）并写入Chassis类
    float scale = 1.0f;
    if (speed_max > kWheelSpeedMax) {
        scale = kWheelSpeedMax / speed_max;
    }
    motion_chassis_ = ChassisMotion(vx * scale, vy * scale, yaw_rate_ * scale);
    yaw_rate_ *= scale;

    // 逐轮解算并pid输出
    for (int i = 0; i < 4u; i++) {
        hello_world::motor::Motor* steer = motor_steer_[i];
        hello_world::motor::Motor* wheel = motor_wheel_[i];
        hello_world::pid::Pid* steer_pid = pid_steer_[i];
        hello_world::pid::Pid* wheel_pid = pid_wheel_[i];

        if (steer == nullptr || wheel == nullptr || steer_pid == nullptr ||
            wheel_pid == nullptr) {
            solution_[i] = WheelSolution();
            continue;
        }

        const float cur_angle = steer->angle();
        const float speed_cmd = sqrtf(vix[i] * vix[i] + viy[i] * viy[i]) * scale; 
        float speed = speed_cmd;
        const float angle_raw = atan2f(viy[i], vix[i]);       
        const bool moving = (speed_cmd >= kSolveSpeedDeadband);

        // 一阶低通
        if (!steer_filter_init_[i]) {
            steer_angle_f_[i] = moving ? angle_raw : cur_angle;
            steer_ref_[i] = cur_angle;
            steer_flip_[i] = false;
            steer_filter_init_[i] = true;
        }

        float angle = cur_angle;
        if (moving) {
            const float k_lpf_steer = (kSteerFilterTau > 1e-4f)
                    ? hello_world::Bound(dt / kSteerFilterTau, 1.0f, 0.0f) : 1.0f;
            steer_angle_f_[i] = hello_world::AngleNormRad( steer_angle_f_[i] +
                k_lpf_steer * hello_world::AngleNormRad(angle_raw - steer_angle_f_[i]));

            // 舵角多解
            const float d_steer = fabsf(hello_world::AngleNormRad(steer_angle_f_[i] - cur_angle));
            if (!steer_flip_[i] && d_steer > PI / 2 + kSteerFlipHyst) {
                steer_flip_[i] = true;
            } else if (steer_flip_[i] && d_steer < PI / 2 - kSteerFlipHyst) {
                steer_flip_[i] = false;
            }

            angle = steer_angle_f_[i] + (steer_flip_[i] ? PI : 0.0f);
            if (steer_flip_[i]) { speed = -speed; }

        } else {
            // 摇杆全零（轮速低于死区）：四个舵轮摆成自转姿态（切向），轮速保持 0
            speed = 0.0f;
            angle = kIdleSteerAngle[i];
        }

        // 目标角速率限制
        const float d_ref = hello_world::Bound(
            hello_world::AngleNormRad(angle - steer_ref_[i]), -kSteerRateMax * dt, kSteerRateMax * dt);
        steer_ref_[i] = hello_world::AngleNormRad(steer_ref_[i] + d_ref);

        solution_[i].steer_angle = steer_ref_[i];
        solution_[i].wheel_speed = speed / kWheelRadius;

        // 舵向角环：目标角平移到合理等效值再进 PID，防止跨零时突变
        const float ref = cur_angle + hello_world::AngleNormRad(steer_ref_[i] - cur_angle);

        // 舵角速度前馈
        float w_steer_ref = 0.0f;
        if (moving && steer_ref_last_init_[i]) {
            w_steer_ref = hello_world::AngleNormRad(steer_ref_[i] - steer_ref_last_[i]) / dt;
        }
        steer_ref_last_[i] = steer_ref_[i];
        steer_ref_last_init_[i] = true;

        const float i_pid = steer_pid->calc(ref, cur_angle, dt) * kSteerCurrPerRaw;
        steer->setInput(i_pid + kSteerVelFfA * w_steer_ref);

        // 驱动轮速度环：目标为输出端角速度，输出为输出端力矩
        wheel->setInput(wheel_pid->calc(solution_[i].wheel_speed, wheel->vel(), dt));
    }
}

void robot::Chassis::recoverSteer(float dt)
{
    for (int i = 0; i < 4u; i++) {
        if (motor_wheel_[i] != nullptr) {
            motor_wheel_[i]->setInput(0.0f);
        }     // 驱动轮不出力
        solution_[i] = WheelSolution();   // solution 置零

        if (motor_steer_[i] == nullptr || pid_steer_[i] == nullptr) {
            continue;
        }

        // 舵角回中：目标 0，输出限幅避免自检时舵机猛甩
        const float cur_angle = motor_steer_[i]->angle();
        const float ref = cur_angle + hello_world::AngleNormRad(-cur_angle);
        const float out = pid_steer_[i]->calc(ref, cur_angle, dt);
        motor_steer_[i]->setInput(hello_world::Bound(out, kSteerCenterOutMax, -kSteerCenterOutMax) * kSteerCurrPerRaw);
    }
}

bool robot::Chassis::isSteerCentered() const
{
    for (int i = 0; i < 4u; i++) {
        if (motor_steer_[i] == nullptr) {
            return false;
        }
        if (fabsf(hello_world::AngleNormRad(motor_steer_[i]->angle())) >
            kSteerCenterTol) {
            return false;
        }
    }
    return true;
}

void robot::Chassis::resetController()
{
    for (int i = 0; i < 4u; i++) {
        if (pid_wheel_[i] != nullptr) {
            pid_wheel_[i]->reset();
        }
        if (pid_steer_[i] != nullptr) {
            pid_steer_[i]->reset();
        }
        // 整形器与前馈状态一并复位：下次进控制时从当前角度重新起步
        steer_filter_init_[i] = false;
        steer_flip_[i] = false;
        steer_ref_last_init_[i] = false;
    }
    delta_f_init_ = false;   // 跟随环 δ 低通也重新起步
    pid_follow_.reset();
    yaw_rate_ = 0.0f;
}

void robot::Chassis::stop()
{
    for (int i = 0; i < 4u; i++) {
        if (motor_wheel_[i] != nullptr) {
            motor_wheel_[i]->setInput(0.0f);
        }
        if (motor_steer_[i] != nullptr) {
            motor_steer_[i]->setInput(0.0f);
        }
        solution_[i] = WheelSolution();
    }
    resetController();
    motion_chassis_.reset();
}

const robot::Chassis::WheelSolution& robot::Chassis::solution(int idx) const{
    HW_ASSERT(0 <= idx && idx < 4, "Error wheel solution idx: %d", idx);
    return solution_[idx];
}

float robot::Chassis::calcYawRate(float w_manual, float dt){
    float w = 0.0f;
    switch (mode_) {
        case ChassisMode::kFollow: {
            // δ 低通
            if (!delta_f_init_) {
                delta_f_ = delta_;
                delta_f_init_ = true;
            }
            const float k_lpf = (kFollowDeltaTau > 1e-4f)
                    ? hello_world::Bound(dt / kFollowDeltaTau, 1.0f, 0.0f) : 1.0f;
            delta_f_ = hello_world::AngleNormRad(
                delta_f_ + k_lpf * hello_world::AngleNormRad(delta_ - delta_f_));
            w = pid_follow_.calc(delta_f_, 0.0f, dt) - kFollowKdGyro * gyro_z_;  // calc内无接口，故暴露在外面
            w = hello_world::Bound(w, kMaxFollowYawRate, -kMaxFollowYawRate);
            break;
        }
        case ChassisMode::kSpin:
            w = hello_world::Bound(6.0f, kMaxSpinRate, -kMaxSpinRate);
            break;
        case ChassisMode::kSeperate:
            w = 0.0f;
            break;
        default:   // 非法模式，保守停机
            w = 0.0f;
            break;
    }
    return w;
}


