/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * pid.h - implementation of the PID regulator
 */
// #ifndef PID_H_
// #define PID_H_

// #include <stdbool.h>
// #include "filter.h"

// #ifdef CONFIG_TARGET_ESP32_S2_DRONE_V1_2
//   #define PID_ROLL_RATE_KP  190.0
//   #define PID_ROLL_RATE_KI  440.0
//   #define PID_ROLL_RATE_KD  2.6
//   #define PID_ROLL_RATE_INTEGRATION_LIMIT    33.3

//   #define PID_PITCH_RATE_KP  190.0
//   #define PID_PITCH_RATE_KI  440.0
//   #define PID_PITCH_RATE_KD  2.6
//   #define PID_PITCH_RATE_INTEGRATION_LIMIT   33.3

//   #define PID_YAW_RATE_KP  120.0
//   #define PID_YAW_RATE_KI  16.7
//   #define PID_YAW_RATE_KD  0.0
//   #define PID_YAW_RATE_INTEGRATION_LIMIT     166.7

//   #define PID_ROLL_KP  5.3
//   #define PID_ROLL_KI  2.5
//   #define PID_ROLL_KD  0.0
//   #define PID_ROLL_INTEGRATION_LIMIT    20.0

//   #define PID_PITCH_KP  5.3
//   #define PID_PITCH_KI  2.5
//   #define PID_PITCH_KD  0.0
//   #define PID_PITCH_INTEGRATION_LIMIT   20.0

//   #define PID_YAW_KP  6.0
//   #define PID_YAW_KI  1.0
//   #define PID_YAW_KD  0.35
//   #define PID_YAW_INTEGRATION_LIMIT     360.0


//   #define DEFAULT_PID_INTEGRATION_LIMIT 5000.0
//   #define DEFAULT_PID_OUTPUT_LIMIT      0.0
// #else
//   #define PID_ROLL_RATE_KP  250.0
//   #define PID_ROLL_RATE_KI  500.0
//   #define PID_ROLL_RATE_KD  2.5
//   #define PID_ROLL_RATE_INTEGRATION_LIMIT    33.3

//   #define PID_PITCH_RATE_KP  250.0
//   #define PID_PITCH_RATE_KI  500.0
//   #define PID_PITCH_RATE_KD  2.5
//   #define PID_PITCH_RATE_INTEGRATION_LIMIT   33.3

//   #define PID_YAW_RATE_KP  120.0
//   #define PID_YAW_RATE_KI  16.7
//   #define PID_YAW_RATE_KD  0.0
//   #define PID_YAW_RATE_INTEGRATION_LIMIT     166.7

//   #define PID_ROLL_KP  5.9
//   #define PID_ROLL_KI  2.9
//   #define PID_ROLL_KD  0.0
//   #define PID_ROLL_INTEGRATION_LIMIT    20.0

//   #define PID_PITCH_KP  5.9
//   #define PID_PITCH_KI  2.9
//   #define PID_PITCH_KD  0.0
//   #define PID_PITCH_INTEGRATION_LIMIT   20.0

//   #define PID_YAW_KP  6.0
//   #define PID_YAW_KI  1.0
//   #define PID_YAW_KD  0.35
//   #define PID_YAW_INTEGRATION_LIMIT     360.0


//   #define DEFAULT_PID_INTEGRATION_LIMIT 5000.0
//   #define DEFAULT_PID_OUTPUT_LIMIT      0.0
// #endif 

// typedef struct
// {
//   float desired;      //< set point
//   float error;        //< error
//   float prevError;    //< previous error
//   float integ;        //< integral
//   float deriv;        //< derivative
//   float kp;           //< proportional gain
//   float ki;           //< integral gain
//   float kd;           //< derivative gain
//   float outP;         //< proportional output (debugging)
//   float outI;         //< integral output (debugging)
//   float outD;         //< derivative output (debugging)
//   float iLimit;       //< integral limit, absolute value. '0' means no limit.
//   float outputLimit;  //< total PID output limit, absolute value. '0' means no limit.
//   float dt;           //< delta-time dt
//   lpf2pData dFilter;  //< filter for D term
//   bool enableDFilter; //< filter for D term enable flag
// } PidObject;

// /**
//  * PID object initialization.
//  *
//  * @param[out] pid   A pointer to the pid object to initialize.
//  * @param[in] desired  The initial set point.
//  * @param[in] kp        The proportional gain
//  * @param[in] ki        The integral gain
//  * @param[in] kd        The derivative gain
//  * @param[in] dt        Delta time since the last call
//  * @param[in] samplingRate Frequency the update will be called
//  * @param[in] cutoffFreq   Frequency to set the low pass filter cutoff at
//  * @param[in] enableDFilter Enable setting for the D lowpass filter
//  */
//  void pidInit(PidObject* pid, const float desired, const float kp,
//               const float ki, const float kd, const float dt,
//               const float samplingRate, const float cutoffFreq,
//               bool enableDFilter);

// /**
//  * Set the integral limit for this PID in deg.
//  *
//  * @param[in] pid   A pointer to the pid object.
//  * @param[in] limit Pid integral swing limit.
//  */
// void pidSetIntegralLimit(PidObject* pid, const float limit);

// /**
//  * Reset the PID error values
//  *
//  * @param[in] pid   A pointer to the pid object.
//  * @param[in] limit Pid integral swing limit.
//  */
// void pidReset(PidObject* pid);

// /**
//  * Update the PID parameters.
//  *
//  * @param[in] pid         A pointer to the pid object.
//  * @param[in] measured    The measured value
//  * @param[in] updateError Set to TRUE if error should be calculated.
//  *                        Set to False if pidSetError() has been used.
//  * @return PID algorithm output
//  */
// float pidUpdate(PidObject* pid, const float measured, const bool updateError);

// /**
//  * Set a new set point for the PID to track.
//  *
//  * @param[in] pid   A pointer to the pid object.
//  * @param[in] angle The new set point
//  */
// void pidSetDesired(PidObject* pid, const float desired);

// /**
//  * Set a new set point for the PID to track.
//  * @return The set point
//  */
// float pidGetDesired(PidObject* pid);

// /**
//  * Find out if PID is active
//  * @return TRUE if active, FALSE otherwise
//  */
// bool pidIsActive(PidObject* pid);

// /**
//  * Set a new error. Use if a special error calculation is needed.
//  *
//  * @param[in] pid   A pointer to the pid object.
//  * @param[in] error The new error
//  */
// void pidSetError(PidObject* pid, const float error);

// /**
//  * Set a new proportional gain for the PID.
//  *
//  * @param[in] pid   A pointer to the pid object.
//  * @param[in] kp    The new proportional gain
//  */
// void pidSetKp(PidObject* pid, const float kp);

// /**
//  * Set a new integral gain for the PID.
//  *
//  * @param[in] pid   A pointer to the pid object.
//  * @param[in] ki    The new integral gain
//  */
// void pidSetKi(PidObject* pid, const float ki);

// /**
//  * Set a new derivative gain for the PID.
//  *
//  * @param[in] pid   A pointer to the pid object.
//  * @param[in] kd    The derivative gain
//  */
// void pidSetKd(PidObject* pid, const float kd);

// /**
//  * Set a new dt gain for the PID. Defaults to IMU_UPDATE_DT upon construction
//  *
//  * @param[in] pid   A pointer to the pid object.
//  * @param[in] dt    Delta time
//  */
// void pidSetDt(PidObject* pid, const float dt);
// #endif /* PID_H_ */

#ifndef PID_H
#define PID_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ===== 常用缺省值（可按工程规模调整） =====
#ifndef DEFAULT_PID_INTEGRATION_LIMIT
#define DEFAULT_PID_INTEGRATION_LIMIT   (300.0f)
#endif

#ifndef DEFAULT_PID_OUTPUT_LIMIT
#define DEFAULT_PID_OUTPUT_LIMIT        (800.0f)
#endif

// ===== 小工具：constrain，若项目里已定义会跳过 =====
#ifndef constrain
#define constrain(x, a, b) ((x) < (a) ? (a) : (((x) > (b)) ? (b) : (x)))
#endif

// ===== 二阶低通(Butterworth)滤波器，给 D 项用 =====
typedef struct {
  // 双二阶（双线性变换）系数
  float b0, b1, b2;
  float a1, a2;
  // 延迟状态
  float x1, x2;
  float y1, y2;
  // 仅用于记录
  float fs;
  float fc;
} lpf2pData;

void  lpf2pInit (lpf2pData* f, float samplingRate, float cutoffFreq);
float lpf2pApply(lpf2pData* f, float x);

// ===== PID 结构体 =====
typedef struct {
  // 误差/状态
  float error;
  float prevError;
  float integ;
  float deriv;
  float prevMeasured;

  // 设定值
  float desired;

  // 增益
  float kp, ki, kd;

  // 限幅
  float iLimit;       // 积分限幅（防积分炸裂）
  float outputLimit;  // 总输出限幅（与混控量纲一致）

  // 时间步长
  float dt;

  // D 项滤波
  bool  enableDFilter;
  lpf2pData dFilter;

  // 选项
  bool  derivOnMeasurement; // true: D 用 -d(measured)/dt
  bool  antiWindup;          // 抗饱和防风up

  // 便于调参观测
  float outP, outI, outD;
} PidObject;

// ===== 接口 =====
void  pidInit(PidObject* pid, float desired, float kp, float ki, float kd,
              float dt, float samplingRate, float cutoffFreq, bool enableDFilter);

float pidUpdate(PidObject* pid, float measured, bool updateError);

void  pidReset(PidObject* pid);

void  pidSetError       (PidObject* pid, float error);
void  pidSetDesired     (PidObject* pid, float desired);
float pidGetDesired     (PidObject* pid);
bool  pidIsActive       (PidObject* pid);

void  pidSetKp          (PidObject* pid, float kp);
void  pidSetKi          (PidObject* pid, float ki);
void  pidSetKd          (PidObject* pid, float kd);
void  pidSetDt          (PidObject* pid, float dt);

void  pidSetIntegralLimit(PidObject* pid, float limit);
void  pidSetOutputLimit  (PidObject* pid, float limit);

// 运行期可切换的选项
void  pidSetDerivOnMeasurement(PidObject* pid, bool enabled);
void  pidSetAntiWindup        (PidObject* pid, bool enabled);

#ifdef __cplusplus
}
#endif

#endif // PID_H
