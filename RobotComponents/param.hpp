/*
 * @Author: Qian Yicheng abused-2@users.noreply.github.com
 * @Date: 2026-11-11 17:15:28
 * @LastEditors: Qian Yicheng abused-2@users.noreply.github.com
 * @LastEditTime: 2026-11-14 22:20:48
 * @FilePath: \steered_skeleton_operate_ai_debug\RobotComponents\param.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*   云台参数   */

// kMotorYawAngleOffset = 1.60879f
// kMotorPitchAngleOffset = 2.86576f       关节零点偏移 (rad)（已按"云台水平=0"标定）

// kYawKp = 8f             
// kYawKi = 3.0f            
// kYawKd = 1.5f              
// kYawOutMax = 7.0f      // 输出力矩上限 (N·m)
// kYawIntegMax = 0.8f    

// kPitchKp = 17f            
// kPitchKi = 3.2f            
// kPitchKd = 1.2f             
// kPitchOutMax = 7.0f    // 输出力矩上限 (N·m)
// kPitchIntegMax = 2.0f      

// kTorqueLimit = 7.0f           // 关节力矩上限(Nm)

// kPitchLowerLimit = -0.3f
// kPitchUpperLimit = 0.58f    // pitch 软件限位（机械行程 -0.2657/+0.5986 各留 0.05 余量）(rad)

// kYawTargetRateLimit = 8.0f
// kPitchTargetRateLimit = 4.0f   // 关节目标角速率上限(rad/s)

/*   底盘参数   */

// kSteerAngleOffset[4] = {2.61346f, -2.06844f, 0.00729f, 1.58518f}    // 舵电机零点标定值

// kWheelKp = 0.35f
// kWheelKi = 0.60f
// kWheelKd = 0.0f
// kWheelOutMax = 2.5f
// kWheelIntegMax = 0.8f

// kSteerKp = 8000.0f
// kSteerKi = 80.0f
// kSteerKd = 400.0f
// kSteerOutMax = 9000.0f
// kSteerIntegMax = 2500.0f

// kFollowKp = 4.0f
// kFollowKi = 0.5f
// kFollowKd = 0.0f            
// kFollowIntegMax = 2.0f    
// kFollowDeltaTau = 0.020f   // 跟随环 δ 低通时间常数 (s) 
// kFollowKdGyro = -0.6f      // 跟随环阻尼系数，作用在底盘陀螺上

// kSolveSpeedDeadband = 0.02f    // 逆解死区：轮心目标线速度低于该值时不转舵
// kWheelSpeedMax = 3.0f          // 单个轮组的线速度上限 (m/s) 超出后整体等比缩放

/*   新增参数   */

// kPitchFrictionComp = 0.25f     // pitch 库仑摩擦补偿力矩 (N·m)，置 0 关闭 
// kPitchFrictionBand = 0.002f    // pitch 摩擦补偿完全退出点 (rad) 
// kPitchFrictionFull = 0.012f    // pitch 摩擦补偿满补偿点 (rad) 

// kPitchGravityA = 0.80f         // pitch 重力前馈 A：水平位托举力矩 (N·m) 
// kPitchGravityB = 0.00f         // pitch 重力前馈 B：偏心相位项 (N·m)

// kSteerFilterTau = 0.020f       // 舵角目标一阶低通时间常数 (s) 
// kSteerRateMax = 20.0f          // 舵角目标速率上限 (rad/s) 
// kSteerFlipHyst = 0.10f         // 舵角翻转滞环半宽 (rad) 
// kSteerVelFfA = 0.03f           // 舵角速度前馈系数 (A per rad/s) 
// kDeltaLeadTime = 0.005f        // 云台系->车系变换的 δ 角速度前馈时间 (s) 

// kYawSpinFfGain = 0.05f         // G8 yaw 反向拖动前馈 (N·m per rad/s)

