/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
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
 * attitude_pid_controller.c: Attitude controler using PID correctors
 */
#include <stdbool.h>
#include <stdint.h>   
#include <math.h>

#include "FreeRTOS.h"

#include "attitude_controller.h"
#include "pid.h"
#include "param.h"
#include "log.h"

// Fallback defaults for PID gains & limits (safe starters)
#ifndef ATTITUDE_RATE
#define ATTITUDE_RATE 500.0f   // If already present in the project, it will not take effect.
#endif

// Rate loop (deg/s), 500 Hz
#ifndef PID_ROLL_RATE_KP
#define PID_ROLL_RATE_KP 250.0f
#endif
#ifndef PID_ROLL_RATE_KI
#define PID_ROLL_RATE_KI 500.0f
#endif
#ifndef PID_ROLL_RATE_KD
#define PID_ROLL_RATE_KD 2.5f
#endif
#ifndef PID_ROLL_RATE_INTEGRATION_LIMIT
#define PID_ROLL_RATE_INTEGRATION_LIMIT 300.0f
#endif

#ifndef PID_PITCH_RATE_KP
#define PID_PITCH_RATE_KP 250.0f
#endif
#ifndef PID_PITCH_RATE_KI
#define PID_PITCH_RATE_KI 500.0f
#endif
#ifndef PID_PITCH_RATE_KD
#define PID_PITCH_RATE_KD 2.5f
#endif
#ifndef PID_PITCH_RATE_INTEGRATION_LIMIT
#define PID_PITCH_RATE_INTEGRATION_LIMIT 300.0f
#endif

#ifndef PID_YAW_RATE_KP
#define PID_YAW_RATE_KP 350.0f
#endif
#ifndef PID_YAW_RATE_KI
#define PID_YAW_RATE_KI 500.0f
#endif
#ifndef PID_YAW_RATE_KD
#define PID_YAW_RATE_KD 2.0f
#endif
#ifndef PID_YAW_RATE_INTEGRATION_LIMIT
#define PID_YAW_RATE_INTEGRATION_LIMIT 300.0f
#endif

//Angle loop (deg), 250 Hz
#ifndef PID_ROLL_KP
#define PID_ROLL_KP 6.0f
#endif
#ifndef PID_ROLL_KI
#define PID_ROLL_KI 3.0f
#endif
#ifndef PID_ROLL_KD
#define PID_ROLL_KD 0.0f
#endif
#ifndef PID_ROLL_INTEGRATION_LIMIT
#define PID_ROLL_INTEGRATION_LIMIT 20.0f
#endif

#ifndef PID_PITCH_KP
#define PID_PITCH_KP 6.0f
#endif
#ifndef PID_PITCH_KI
#define PID_PITCH_KI 3.0f
#endif
#ifndef PID_PITCH_KD
#define PID_PITCH_KD 0.0f
#endif
#ifndef PID_PITCH_INTEGRATION_LIMIT
#define PID_PITCH_INTEGRATION_LIMIT 20.0f
#endif

#ifndef PID_YAW_KP
#define PID_YAW_KP 2.0f
#endif
#ifndef PID_YAW_KI
#define PID_YAW_KI 1.0f
#endif
#ifndef PID_YAW_KD
#define PID_YAW_KD 0.0f
#endif
#ifndef PID_YAW_INTEGRATION_LIMIT
#define PID_YAW_INTEGRATION_LIMIT 10.0f
#endif


#define ATTITUDE_LPF_CUTOFF_FREQ      15.0f
#define ATTITUDE_LPF_ENABLE false
#define ATTITUDE_RATE_LPF_CUTOFF_FREQ 30.0f
#define ATTITUDE_RATE_LPF_ENABLE false


static inline int16_t saturateSignedInt16(float in)
{
  // don't use INT16_MIN, because later we may negate it, which won't work for that value.
  if (in > INT16_MAX)
    return INT16_MAX;
  else if (in < -INT16_MAX)
    return -INT16_MAX;
  else
    return (int16_t)in;
}

PidObject pidRollRate;
PidObject pidPitchRate;
PidObject pidYawRate;
PidObject pidRoll;
PidObject pidPitch;
PidObject pidYaw;

static int16_t rollOutput;
static int16_t pitchOutput;
static int16_t yawOutput;

static bool isInit;

void attitudeControllerInit(const float updateDt)
{
  if(isInit)
    return;

  //TODO: get parameters from configuration manager instead
  pidInit(&pidRollRate,  0, PID_ROLL_RATE_KP,  PID_ROLL_RATE_KI,  PID_ROLL_RATE_KD,
      updateDt, ATTITUDE_RATE, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);
  pidInit(&pidPitchRate, 0, PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD,
      updateDt, ATTITUDE_RATE, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);
  pidInit(&pidYawRate,   0, PID_YAW_RATE_KP,   PID_YAW_RATE_KI,   PID_YAW_RATE_KD,
      updateDt, ATTITUDE_RATE, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);

  pidSetIntegralLimit(&pidRollRate,  PID_ROLL_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidPitchRate, PID_PITCH_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidYawRate,   PID_YAW_RATE_INTEGRATION_LIMIT);

  pidInit(&pidRoll,  0, PID_ROLL_KP,  PID_ROLL_KI,  PID_ROLL_KD,  updateDt,
      ATTITUDE_RATE, ATTITUDE_LPF_CUTOFF_FREQ, ATTITUDE_LPF_ENABLE);
  pidInit(&pidPitch, 0, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, updateDt,
      ATTITUDE_RATE, ATTITUDE_LPF_CUTOFF_FREQ, ATTITUDE_LPF_ENABLE);
  pidInit(&pidYaw,   0, PID_YAW_KP,   PID_YAW_KI,   PID_YAW_KD,   updateDt,
      ATTITUDE_RATE, ATTITUDE_LPF_CUTOFF_FREQ, ATTITUDE_LPF_ENABLE);

  pidSetIntegralLimit(&pidRoll,  PID_ROLL_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidPitch, PID_PITCH_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidYaw,   PID_YAW_INTEGRATION_LIMIT);

  isInit = true;
}

bool attitudeControllerTest()
{
  return isInit;
}

void attitudeControllerCorrectRatePID(
       float rollRateActual, float pitchRateActual, float yawRateActual,
       float rollRateDesired, float pitchRateDesired, float yawRateDesired)
{
  pidSetDesired(&pidRollRate, rollRateDesired);
  rollOutput = saturateSignedInt16(pidUpdate(&pidRollRate, rollRateActual, true));

  pidSetDesired(&pidPitchRate, pitchRateDesired);
  pitchOutput = saturateSignedInt16(pidUpdate(&pidPitchRate, pitchRateActual, true));

  pidSetDesired(&pidYawRate, yawRateDesired);
  yawOutput = saturateSignedInt16(pidUpdate(&pidYawRate, yawRateActual, true));
}

void attitudeControllerCorrectAttitudePID(
       float eulerRollActual, float eulerPitchActual, float eulerYawActual,
       float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
       float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired)
{
  pidSetDesired(&pidRoll, eulerRollDesired);
  *rollRateDesired = pidUpdate(&pidRoll, eulerRollActual, true);

  // Update PID for pitch axis
  pidSetDesired(&pidPitch, eulerPitchDesired);
  *pitchRateDesired = pidUpdate(&pidPitch, eulerPitchActual, true);

  // Update PID for yaw axis
  float yawError;
  yawError = eulerYawDesired - eulerYawActual;
  if (yawError > 180.0f)
    yawError -= 360.0f;
  else if (yawError < -180.0f)
    yawError += 360.0f;
  pidSetError(&pidYaw, yawError);
  *yawRateDesired = pidUpdate(&pidYaw, eulerYawActual, false);
}

void attitudeControllerResetRollAttitudePID(void)
{
    pidReset(&pidRoll);
}

void attitudeControllerResetPitchAttitudePID(void)
{
    pidReset(&pidPitch);
}

void attitudeControllerResetAllPID(void)
{
  pidReset(&pidRoll);
  pidReset(&pidPitch);
  pidReset(&pidYaw);
  pidReset(&pidRollRate);
  pidReset(&pidPitchRate);
  pidReset(&pidYawRate);
}

void attitudeControllerGetActuatorOutput(int16_t* roll, int16_t* pitch, int16_t* yaw)
{
  *roll = rollOutput;
  *pitch = pitchOutput;
  *yaw = yawOutput;
}

LOG_GROUP_START(pid_attitude)
LOG_ADD(LOG_FLOAT, roll_outP, &pidRoll.outP)
LOG_ADD(LOG_FLOAT, roll_outI, &pidRoll.outI)
LOG_ADD(LOG_FLOAT, roll_outD, &pidRoll.outD)
LOG_ADD(LOG_FLOAT, pitch_outP, &pidPitch.outP)
LOG_ADD(LOG_FLOAT, pitch_outI, &pidPitch.outI)
LOG_ADD(LOG_FLOAT, pitch_outD, &pidPitch.outD)
LOG_ADD(LOG_FLOAT, yaw_outP, &pidYaw.outP)
LOG_ADD(LOG_FLOAT, yaw_outI, &pidYaw.outI)
LOG_ADD(LOG_FLOAT, yaw_outD, &pidYaw.outD)
LOG_GROUP_STOP(pid_attitude)

LOG_GROUP_START(pid_rate)
LOG_ADD(LOG_FLOAT, roll_outP, &pidRollRate.outP)
LOG_ADD(LOG_FLOAT, roll_outI, &pidRollRate.outI)
LOG_ADD(LOG_FLOAT, roll_outD, &pidRollRate.outD)
LOG_ADD(LOG_FLOAT, pitch_outP, &pidPitchRate.outP)
LOG_ADD(LOG_FLOAT, pitch_outI, &pidPitchRate.outI)
LOG_ADD(LOG_FLOAT, pitch_outD, &pidPitchRate.outD)
LOG_ADD(LOG_FLOAT, yaw_outP, &pidYawRate.outP)
LOG_ADD(LOG_FLOAT, yaw_outI, &pidYawRate.outI)
LOG_ADD(LOG_FLOAT, yaw_outD, &pidYawRate.outD)
LOG_GROUP_STOP(pid_rate)

PARAM_GROUP_START(pid_attitude)
PARAM_ADD(PARAM_FLOAT, roll_kp, &pidRoll.kp)
PARAM_ADD(PARAM_FLOAT, roll_ki, &pidRoll.ki)
PARAM_ADD(PARAM_FLOAT, roll_kd, &pidRoll.kd)
PARAM_ADD(PARAM_FLOAT, pitch_kp, &pidPitch.kp)
PARAM_ADD(PARAM_FLOAT, pitch_ki, &pidPitch.ki)
PARAM_ADD(PARAM_FLOAT, pitch_kd, &pidPitch.kd)
PARAM_ADD(PARAM_FLOAT, yaw_kp, &pidYaw.kp)
PARAM_ADD(PARAM_FLOAT, yaw_ki, &pidYaw.ki)
PARAM_ADD(PARAM_FLOAT, yaw_kd, &pidYaw.kd)
PARAM_GROUP_STOP(pid_attitude)

PARAM_GROUP_START(pid_rate)
PARAM_ADD(PARAM_FLOAT, roll_kp, &pidRollRate.kp)
PARAM_ADD(PARAM_FLOAT, roll_ki, &pidRollRate.ki)
PARAM_ADD(PARAM_FLOAT, roll_kd, &pidRollRate.kd)
PARAM_ADD(PARAM_FLOAT, pitch_kp, &pidPitchRate.kp)
PARAM_ADD(PARAM_FLOAT, pitch_ki, &pidPitchRate.ki)
PARAM_ADD(PARAM_FLOAT, pitch_kd, &pidPitchRate.kd)
PARAM_ADD(PARAM_FLOAT, yaw_kp, &pidYawRate.kp)
PARAM_ADD(PARAM_FLOAT, yaw_ki, &pidYawRate.ki)
PARAM_ADD(PARAM_FLOAT, yaw_kd, &pidYawRate.kd)
PARAM_GROUP_STOP(pid_rate)

// #include <stdbool.h>
// #include <stdint.h>
// #include <math.h>

// #include "FreeRTOS.h"

// #include "attitude_controller.h"
// #include "pid.h"
// #include "param.h"
// #include "log.h"

// // ================== 可调开关/默认值 ==================
// // 建议：外环 D 关闭，内环 D 开启并低通 ~110 Hz
// #define ATTITUDE_LPF_CUTOFF_FREQ       15.0f
// #define ATTITUDE_LPF_ENABLE            false
// #define ATTITUDE_RATE_LPF_CUTOFF_FREQ  110.0f
// #define ATTITUDE_RATE_LPF_ENABLE       true

// // 速率死区（抑制 MPU6050 零偏，单位：deg/s）
// static float rateDeadbandDps = 2.5f;

// // 内环输出限幅（量纲=你的 rate 命令量纲，CF 风格可用 800）
// static float rateOutLimRoll  = 200.0f;
// static float rateOutLimPitch = 200.0f;
// static float rateOutLimYaw   = 100.0f;

// // 外环输出的是“期望角速率”的限幅（deg/s）
// static float angOutLimRoll   = 180.0f;
// static float angOutLimPitch  = 180.0f;
// static float angOutLimYaw    = 160.0f;

// // （如需：积分限幅仍沿用你在 pidInit 后设置的 *_INTEGRATION_LIMIT）

// static inline int16_t saturateSignedInt16(float in)
// {
//   if (in > INT16_MAX)      return INT16_MAX;
//   else if (in < -INT16_MAX) return -INT16_MAX;
//   else                      return (int16_t)in;
// }

// static inline float deadbandf(float v, float db)
// {
//   return (fabsf(v) < db) ? 0.0f : v;
// }

// // ================== PID 实例 ==================
// PidObject pidRollRate;
// PidObject pidPitchRate;
// PidObject pidYawRate;
// PidObject pidRoll;
// PidObject pidPitch;
// PidObject pidYaw;

// static int16_t rollOutput;
// static int16_t pitchOutput;
// static int16_t yawOutput;

// static bool isInit;

// // ================== 初始化 ==================
// void attitudeControllerInit(const float updateDt)
// {
//   if (isInit) return;

//   // ==== 内环（角速率）====
//   pidInit(&pidRollRate,  0, PID_ROLL_RATE_KP,  PID_ROLL_RATE_KI,  PID_ROLL_RATE_KD,
//           updateDt, ATTITUDE_RATE, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);
//   pidInit(&pidPitchRate, 0, PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD,
//           updateDt, ATTITUDE_RATE, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);
//   pidInit(&pidYawRate,   0, PID_YAW_RATE_KP,   PID_YAW_RATE_KI,   PID_YAW_RATE_KD,
//           updateDt, ATTITUDE_RATE, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);

//   // 抗饱和 / 基于测量微分 / 输出限幅
//   pidSetAntiWindup(&pidRollRate,  true);
//   pidSetAntiWindup(&pidPitchRate, true);
//   pidSetAntiWindup(&pidYawRate,   true);

//   pidSetDerivOnMeasurement(&pidRollRate,  true);
//   pidSetDerivOnMeasurement(&pidPitchRate, true);
//   pidSetDerivOnMeasurement(&pidYawRate,   true);

//   pidSetIntegralLimit(&pidRollRate,  PID_ROLL_RATE_INTEGRATION_LIMIT);
//   pidSetIntegralLimit(&pidPitchRate, PID_PITCH_RATE_INTEGRATION_LIMIT);
//   pidSetIntegralLimit(&pidYawRate,   PID_YAW_RATE_INTEGRATION_LIMIT);

//   pidSetOutputLimit(&pidRollRate,  rateOutLimRoll);
//   pidSetOutputLimit(&pidPitchRate, rateOutLimPitch);
//   pidSetOutputLimit(&pidYawRate,   rateOutLimYaw);

//   // ==== 外环（角度 → 期望角速率）====
//   pidInit(&pidRoll,  0, PID_ROLL_KP,  PID_ROLL_KI,  PID_ROLL_KD,
//           updateDt, ATTITUDE_RATE, ATTITUDE_LPF_CUTOFF_FREQ, ATTITUDE_LPF_ENABLE);
//   pidInit(&pidPitch, 0, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD,
//           updateDt, ATTITUDE_RATE, ATTITUDE_LPF_CUTOFF_FREQ, ATTITUDE_LPF_ENABLE);
//   pidInit(&pidYaw,   0, PID_YAW_KP,   PID_YAW_KI,   PID_YAW_KD,
//           updateDt, ATTITUDE_RATE, ATTITUDE_LPF_CUTOFF_FREQ, ATTITUDE_LPF_ENABLE);

//   // 外环通常不开 D；仍开启抗饱和更稳
//   pidSetAntiWindup(&pidRoll,  true);
//   pidSetAntiWindup(&pidPitch, true);
//   pidSetAntiWindup(&pidYaw,   true);

//   pidSetIntegralLimit(&pidRoll,  PID_ROLL_INTEGRATION_LIMIT);
//   pidSetIntegralLimit(&pidPitch, PID_PITCH_INTEGRATION_LIMIT);
//   pidSetIntegralLimit(&pidYaw,   PID_YAW_INTEGRATION_LIMIT);

//   // 外环的输出限幅（限制为“期望角速率”的上限）
//   pidSetOutputLimit(&pidRoll,  angOutLimRoll);
//   pidSetOutputLimit(&pidPitch, angOutLimPitch);
//   pidSetOutputLimit(&pidYaw,   angOutLimYaw);

//   isInit = true;
// }

// bool attitudeControllerTest()
// {
//   return isInit;
// }

// // ================== Rate 内环更新 ==================
// void attitudeControllerCorrectRatePID(
//        float rollRateActual, float pitchRateActual, float yawRateActual,
//        float rollRateDesired, float pitchRateDesired, float yawRateDesired)
// {
//   // 陀螺速率死区，抑制零偏引起的小电机纠偏
//   rollRateActual  = deadbandf(rollRateActual,  rateDeadbandDps);
//   pitchRateActual = deadbandf(pitchRateActual, rateDeadbandDps);
//   yawRateActual   = deadbandf(yawRateActual,   rateDeadbandDps);

//   pidSetDesired(&pidRollRate, rollRateDesired);
//   rollOutput = saturateSignedInt16(pidUpdate(&pidRollRate, rollRateActual, true));

//   pidSetDesired(&pidPitchRate, pitchRateDesired);
//   pitchOutput = saturateSignedInt16(pidUpdate(&pidPitchRate, pitchRateActual, true));

//   pidSetDesired(&pidYawRate, yawRateDesired);
//   yawOutput = saturateSignedInt16(pidUpdate(&pidYawRate, yawRateActual, true));
// }

// // ================== Attitude 外环更新 ==================
// void attitudeControllerCorrectAttitudePID(
//        float eulerRollActual, float eulerPitchActual, float eulerYawActual,
//        float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
//        float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired)
// {
//   pidSetDesired(&pidRoll, eulerRollDesired);
//   *rollRateDesired = pidUpdate(&pidRoll, eulerRollActual, true);

//   pidSetDesired(&pidPitch, eulerPitchDesired);
//   *pitchRateDesired = pidUpdate(&pidPitch, eulerPitchActual, true);

//   // Yaw 误差在 [-180,180] 内包角
//   float yawError = eulerYawDesired - eulerYawActual;
//   if (yawError > 180.0f)      yawError -= 360.0f;
//   else if (yawError < -180.0f) yawError += 360.0f;

//   pidSetError(&pidYaw, yawError);
//   *yawRateDesired = pidUpdate(&pidYaw, eulerYawActual, false);
// }

// // ================== 复位/输出 ==================
// void attitudeControllerResetRollAttitudePID(void)  { pidReset(&pidRoll); }
// void attitudeControllerResetPitchAttitudePID(void) { pidReset(&pidPitch); }

// void attitudeControllerResetAllPID(void)
// {
//   pidReset(&pidRoll);
//   pidReset(&pidPitch);
//   pidReset(&pidYaw);
//   pidReset(&pidRollRate);
//   pidReset(&pidPitchRate);
//   pidReset(&pidYawRate);
// }

// void attitudeControllerGetActuatorOutput(int16_t* roll, int16_t* pitch, int16_t* yaw)
// {
//   *roll  = rollOutput;
//   *pitch = pitchOutput;
//   *yaw   = yawOutput;
// }

// // ================== 日志 ==================
// LOG_GROUP_START(pid_attitude)
// LOG_ADD(LOG_FLOAT, roll_outP,  &pidRoll.outP)
// LOG_ADD(LOG_FLOAT, roll_outI,  &pidRoll.outI)
// LOG_ADD(LOG_FLOAT, roll_outD,  &pidRoll.outD)
// LOG_ADD(LOG_FLOAT, pitch_outP, &pidPitch.outP)
// LOG_ADD(LOG_FLOAT, pitch_outI, &pidPitch.outI)
// LOG_ADD(LOG_FLOAT, pitch_outD, &pidPitch.outD)
// LOG_ADD(LOG_FLOAT, yaw_outP,   &pidYaw.outP)
// LOG_ADD(LOG_FLOAT, yaw_outI,   &pidYaw.outI)
// LOG_ADD(LOG_FLOAT, yaw_outD,   &pidYaw.outD)
// LOG_GROUP_STOP(pid_attitude)

// LOG_GROUP_START(pid_rate)
// LOG_ADD(LOG_FLOAT, roll_outP,  &pidRollRate.outP)
// LOG_ADD(LOG_FLOAT, roll_outI,  &pidRollRate.outI)
// LOG_ADD(LOG_FLOAT, roll_outD,  &pidRollRate.outD)
// LOG_ADD(LOG_FLOAT, pitch_outP, &pidPitchRate.outP)
// LOG_ADD(LOG_FLOAT, pitch_outI, &pidPitchRate.outI)
// LOG_ADD(LOG_FLOAT, pitch_outD, &pidPitchRate.outD)
// LOG_ADD(LOG_FLOAT, yaw_outP,   &pidYawRate.outP)
// LOG_ADD(LOG_FLOAT, yaw_outI,   &pidYawRate.outI)
// LOG_ADD(LOG_FLOAT, yaw_outD,   &pidYawRate.outD)
// LOG_GROUP_STOP(pid_rate)

// // ================== 参数暴露（K/KI/KD + 限幅/死区） ==================
// PARAM_GROUP_START(pid_attitude)
// PARAM_ADD(PARAM_FLOAT, roll_kp, &pidRoll.kp)
// PARAM_ADD(PARAM_FLOAT, roll_ki, &pidRoll.ki)
// PARAM_ADD(PARAM_FLOAT, roll_kd, &pidRoll.kd)
// PARAM_ADD(PARAM_FLOAT, pitch_kp, &pidPitch.kp)
// PARAM_ADD(PARAM_FLOAT, pitch_ki, &pidPitch.ki)
// PARAM_ADD(PARAM_FLOAT, pitch_kd, &pidPitch.kd)
// PARAM_ADD(PARAM_FLOAT, yaw_kp, &pidYaw.kp)
// PARAM_ADD(PARAM_FLOAT, yaw_ki, &pidYaw.ki)
// PARAM_ADD(PARAM_FLOAT, yaw_kd, &pidYaw.kd)
// // 外环输出限幅（期望角速率上限）
// PARAM_ADD(PARAM_FLOAT, roll_out_lim,  &angOutLimRoll)
// PARAM_ADD(PARAM_FLOAT, pitch_out_lim, &angOutLimPitch)
// PARAM_ADD(PARAM_FLOAT, yaw_out_lim,   &angOutLimYaw)
// PARAM_GROUP_STOP(pid_attitude)

// PARAM_GROUP_START(pid_rate)
// PARAM_ADD(PARAM_FLOAT, roll_kp, &pidRollRate.kp)
// PARAM_ADD(PARAM_FLOAT, roll_ki, &pidRollRate.ki)
// PARAM_ADD(PARAM_FLOAT, roll_kd, &pidRollRate.kd)
// PARAM_ADD(PARAM_FLOAT, pitch_kp, &pidPitchRate.kp)
// PARAM_ADD(PARAM_FLOAT, pitch_ki, &pidPitchRate.ki)
// PARAM_ADD(PARAM_FLOAT, pitch_kd, &pidPitchRate.kd)
// PARAM_ADD(PARAM_FLOAT, yaw_kp, &pidYawRate.kp)
// PARAM_ADD(PARAM_FLOAT, yaw_ki, &pidYawRate.ki)
// PARAM_ADD(PARAM_FLOAT, yaw_kd, &pidYawRate.kd)
// // 内环输出限幅
// PARAM_ADD(PARAM_FLOAT, roll_out_lim,  &rateOutLimRoll)
// PARAM_ADD(PARAM_FLOAT, pitch_out_lim, &rateOutLimPitch)
// PARAM_ADD(PARAM_FLOAT, yaw_out_lim,   &rateOutLimYaw)
// // 速率死区
// PARAM_ADD(PARAM_FLOAT, rate_deadband_dps, &rateDeadbandDps)
// PARAM_GROUP_STOP(pid_rate)
