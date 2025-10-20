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
 * pid.c - implementation of the PID regulator
 */

// #include "pid.h"
// #include "num.h"
// #include <math.h>
// #include <float.h>

// void pidInit(PidObject* pid, const float desired, const float kp,
//              const float ki, const float kd, const float dt,
//              const float samplingRate, const float cutoffFreq,
//              bool enableDFilter)
// {
//   pid->error         = 0;
//   pid->prevError     = 0;
//   pid->integ         = 0;
//   pid->deriv         = 0;
//   pid->desired       = desired;
//   pid->kp            = kp;
//   pid->ki            = ki;
//   pid->kd            = kd;
//   pid->iLimit        = DEFAULT_PID_INTEGRATION_LIMIT;
//   pid->outputLimit   = DEFAULT_PID_OUTPUT_LIMIT;
//   pid->dt            = dt;
//   pid->enableDFilter = enableDFilter;
//   if (pid->enableDFilter)
//   {
//     lpf2pInit(&pid->dFilter, samplingRate, cutoffFreq);
//   }
// }

// float pidUpdate(PidObject* pid, const float measured, const bool updateError)
// {
//     float output = 0.0f;

//     if (updateError)
//     {
//         pid->error = pid->desired - measured;
//     }

//     pid->outP = pid->kp * pid->error;
//     output += pid->outP;

//     float deriv = (pid->error - pid->prevError) / pid->dt;
//     if (pid->enableDFilter)
//     {
//       pid->deriv = lpf2pApply(&pid->dFilter, deriv);
//     } else {
//       pid->deriv = deriv;
//     }
//     if (isnan(pid->deriv)) {
//       pid->deriv = 0;
//     }
//     pid->outD = pid->kd * pid->deriv;
//     output += pid->outD;

//     pid->integ += pid->error * pid->dt;

//     // Constrain the integral (unless the iLimit is zero)
//     if(pid->iLimit != 0)
//     {
//     	pid->integ = constrain(pid->integ, -pid->iLimit, pid->iLimit);
//     }

//     pid->outI = pid->ki * pid->integ;
//     output += pid->outI;

//     // Constrain the total PID output (unless the outputLimit is zero)
//     if(pid->outputLimit != 0)
//     {
//       output = constrain(output, -pid->outputLimit, pid->outputLimit);
//     }


//     pid->prevError = pid->error;

//     return output;
// }

// void pidSetIntegralLimit(PidObject* pid, const float limit) {
//     pid->iLimit = limit;
// }


// void pidReset(PidObject* pid)
// {
//   pid->error     = 0;
//   pid->prevError = 0;
//   pid->integ     = 0;
//   pid->deriv     = 0;
// }

// void pidSetError(PidObject* pid, const float error)
// {
//   pid->error = error;
// }

// void pidSetDesired(PidObject* pid, const float desired)
// {
//   pid->desired = desired;
// }

// float pidGetDesired(PidObject* pid)
// {
//   return pid->desired;
// }

// bool pidIsActive(PidObject* pid)
// {
//   bool isActive = true;

//   if (pid->kp < 0.0001f && pid->ki < 0.0001f && pid->kd < 0.0001f)
//   {
//     isActive = false;
//   }

//   return isActive;
// }

// void pidSetKp(PidObject* pid, const float kp)
// {
//   pid->kp = kp;
// }

// void pidSetKi(PidObject* pid, const float ki)
// {
//   pid->ki = ki;
// }

// void pidSetKd(PidObject* pid, const float kd)
// {
//   pid->kd = kd;
// }
// void pidSetDt(PidObject* pid, const float dt) {
//     pid->dt = dt;
// }

#include "pid.h"
#include <math.h>
#include <float.h>

// ========== 二阶低通滤波器实现（Butterworth, bilinear） ==========
void lpf2pInit(lpf2pData* f, float samplingRate, float cutoffFreq)
{
  // 防御：fc 合法性
  if (!f || samplingRate <= 0.0f || cutoffFreq <= 0.0f || cutoffFreq >= 0.5f * samplingRate) {
    // 退化为直通
    f->b0 = 1.0f; f->b1 = 0.0f; f->b2 = 0.0f;
    f->a1 = 0.0f; f->a2 = 0.0f;
  } else {
    // 二阶巴特沃斯系数（常用公式）
    const float fs = samplingRate;
    const float fc = cutoffFreq;
    const float K  = tanf((float)M_PI * fc / fs);
    const float K2 = K * K;
    const float sqrt2 = 1.41421356237f;

    const float norm = 1.0f / (1.0f + sqrt2 * K + K2);

    f->b0 = K2 * norm;
    f->b1 = 2.0f * f->b0;
    f->b2 = f->b0;

    f->a1 = 2.0f * (K2 - 1.0f) * norm;
    f->a2 = (1.0f - sqrt2 * K + K2) * norm;
  }

  f->x1 = f->x2 = 0.0f;
  f->y1 = f->y2 = 0.0f;
  f->fs = samplingRate;
  f->fc = cutoffFreq;
}

float lpf2pApply(lpf2pData* f, float x)
{
  if (!f) return x;

  // 直接型 I
  float y = f->b0 * x + f->b1 * f->x1 + f->b2 * f->x2
            - f->a1 * f->y1 - f->a2 * f->y2;

  // 更新状态
  f->x2 = f->x1; f->x1 = x;
  f->y2 = f->y1; f->y1 = y;

  if (isnan(y) || isinf(y)) return 0.0f;
  return y;
}

// ========================== PID ==========================
void pidInit(PidObject* pid, const float desired, const float kp,
             const float ki, const float kd, const float dt,
             const float samplingRate, const float cutoffFreq,
             bool enableDFilter)
{
  pid->error         = 0.0f;
  pid->prevError     = 0.0f;
  pid->integ         = 0.0f;
  pid->deriv         = 0.0f;
  pid->prevMeasured  = 0.0f;

  pid->desired       = desired;

  pid->kp            = kp;
  pid->ki            = ki;
  pid->kd            = kd;

  pid->iLimit        = DEFAULT_PID_INTEGRATION_LIMIT;
  pid->outputLimit   = DEFAULT_PID_OUTPUT_LIMIT;

  pid->dt            = dt;

  pid->enableDFilter = enableDFilter;
  pid->derivOnMeasurement = true;  // 推荐：D 用测量微分，抗 setpoint 阶跃
  pid->antiWindup    = true;

  pid->outP = pid->outI = pid->outD = 0.0f;

  if (pid->enableDFilter) {
    lpf2pInit(&pid->dFilter, samplingRate, cutoffFreq);
  } else {
    // 仍然初始化，系数将是直通（不使用也无妨）
    lpf2pInit(&pid->dFilter, samplingRate, 0.0f);
  }
}

float pidUpdate(PidObject* pid, const float measured, const bool updateError)
{
  float output = 0.0f;

  if (updateError) {
    pid->error = pid->desired - measured;
  }

  // ---- P ----
  pid->outP = pid->kp * pid->error;
  output += pid->outP;

  // ---- D ----
  float rawDeriv;
  if (pid->derivOnMeasurement) {
    // D 只看测量变化，避免目标阶跃带来的 D 冲击
    rawDeriv = -(measured - pid->prevMeasured) / pid->dt;
  } else {
    rawDeriv = (pid->error - pid->prevError) / pid->dt;
  }

  pid->deriv = pid->enableDFilter ? lpf2pApply(&pid->dFilter, rawDeriv) : rawDeriv;
  if (isnan(pid->deriv) || isinf(pid->deriv)) pid->deriv = 0.0f;

  pid->outD = pid->kd * pid->deriv;
  output += pid->outD;

  // ---- I（带抗饱和）----
  // 预测加入积分后的输出，用于判断是否会顶到限幅
  float preSatOutput = output + pid->ki * (pid->integ + pid->error * pid->dt);

  bool hasLimit = (pid->outputLimit != 0.0f);
  bool atUpper  = hasLimit && (preSatOutput >  pid->outputLimit - 1e-6f);
  bool atLower  = hasLimit && (preSatOutput < -pid->outputLimit + 1e-6f);
  bool sameDir  = ((pid->error > 0.0f && atUpper) || (pid->error < 0.0f && atLower));

  if (!pid->antiWindup || !sameDir) {
    pid->integ += pid->error * pid->dt;

    if (pid->iLimit != 0.0f) {
      pid->integ = constrain(pid->integ, -pid->iLimit, pid->iLimit);
    }
  }

  pid->outI = pid->ki * pid->integ;
  output += pid->outI;

  // ---- 总输出限幅 ----
  if (pid->outputLimit != 0.0f) {
    output = constrain(output, -pid->outputLimit, pid->outputLimit);
  }

  // ---- 存档 ----
  pid->prevError    = pid->error;
  pid->prevMeasured = measured;

  return output;
}

void pidReset(PidObject* pid)
{
  pid->error     = 0.0f;
  pid->prevError = 0.0f;
  pid->integ     = 0.0f;
  pid->deriv     = 0.0f;
  pid->prevMeasured = 0.0f;

  pid->outP = pid->outI = pid->outD = 0.0f;
}

void pidSetIntegralLimit(PidObject* pid, const float limit) { pid->iLimit = limit; }
void pidSetOutputLimit  (PidObject* pid, const float limit) { pid->outputLimit = limit; }

void pidSetError  (PidObject* pid, const float error)  { pid->error   = error; }
void pidSetDesired(PidObject* pid, const float desired){ pid->desired = desired; }
float pidGetDesired(PidObject* pid)                    { return pid->desired; }

bool pidIsActive(PidObject* pid)
{
  return !(
    fabsf(pid->kp) < 0.0001f &&
    fabsf(pid->ki) < 0.0001f &&
    fabsf(pid->kd) < 0.0001f
  );
}

void pidSetKp(PidObject* pid, const float kp) { pid->kp = kp; }
void pidSetKi(PidObject* pid, const float ki) { pid->ki = ki; }
void pidSetKd(PidObject* pid, const float kd) { pid->kd = kd; }
void pidSetDt(PidObject* pid, const float dt) { pid->dt = dt; }

void pidSetDerivOnMeasurement(PidObject* pid, bool enabled) { pid->derivOnMeasurement = enabled; }
void pidSetAntiWindup        (PidObject* pid, bool enabled) { pid->antiWindup         = enabled; }
