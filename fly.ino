// File: quad_s3_gpio3456_user_mix_smooth_zero.ino
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>
#include <Wire.h>  // IMU 用到

/************ MPU6050 引脚/参数 ************/
static const int SDA_PIN = 11;   // I2C0_SDA (GPIO11)
static const int SCL_PIN = 10;   // I2C0_SCL (GPIO10)
static const int INT_PIN = 12;   // MPU_INT (GPIO12)
static uint32_t I2C_SPEED = 100000;
static uint8_t  MPU_ADDR  = 0x68;

// 姿态解算常量（±250 dps、±2g）
static const float GYRO_SENS = 131.0f;   // LSB per (deg/s)
static const float ACC_SENS  = 16384.0f; // LSB per g
static const float ALPHA     = 0.98f;    // 互补滤波权重（0.95~0.99 均可）
static bool g_mpu_ok = false;

/************ Wi-Fi AP 配置 ************/
static const char*   AP_SSID   = "UAV-CTRL";
static const char*   AP_PASS   = "12345678";
static const uint16_t UDP_PORT = 14550;
static const uint8_t  WIFI_CH  = 6;

/************ 电机引脚（按你: 1~4 顺时针=GPIO 3/4/5/6） ************/
static const int M1_PIN = 3;  // 电机1 → GPIO3
static const int M2_PIN = 4;  // 电机2 → GPIO4
static const int M3_PIN = 5;  // 电机3 → GPIO5
static const int M4_PIN = 6;  // 电机4 → GPIO6

/************ 全局限值（ESC 常用区间） ************/
static const int THR_MIN_US = 0;   // 你当前设定
static const int THR_MAX_US = 2250;

/************ 全局“目标”变量（你来改） ************/
int g_thr_cmd = THR_MIN_US;   // 基准油门（电脑端 T= 或 CSV 第1项）
int g_cmd_up=0,g_cmd_down=0,g_cmd_left=0,g_cmd_right=0,g_cmd_forward=0,g_cmd_back=0;
int g_m1 = THR_MIN_US, g_m2 = THR_MIN_US, g_m3 = THR_MIN_US, g_m4 = THR_MIN_US;

/************ === 新增：全局姿态变量（供你的算法使用） === ************/
float g_roll_deg  = 0.0f;   // 翻滚（X）
float g_pitch_deg = 0.0f;   // 俯仰（Y）
float g_yaw_deg   = 0.0f;   // 偏航（Z）——无磁力计，随时间会漂移
static uint32_t last_att_us = 0;

/************ 工具 ************/
static inline int clampi(int v, int lo, int hi){ return v<lo?lo:(v>hi?hi:v); }

//*************************************************************************************PID******************************************************************************
// 单函数三轴版 PID（设定角=0°）
// axisSel: 0=roll(翻滚)，1=pitch(俯仰)，2=yaw(偏航)
// 返回：增量推力(µs，正=应增加，负=应减少)
float pidAngleDelta2(float angle_deg, float Kp, float Ki, float Kd, int axisSel) {
  // 规范化轴索引
  int i = 0;
  if (axisSel == 1) i = 1;
  else if (axisSel == 2) i = 2;

  // —— 每个轴独立的静态状态 —— //
  static float     I[3]       = {0, 0, 0};     // 积分项
  static float     prev[3]    = {0, 0, 0};     // 上次角度(°)
  static float     Df[3]      = {0, 0, 0};     // 低通后的 D 项
  static uint32_t  last_us[3] = {0, 0, 0};     // 上次时间戳

  // 可调常量
  const float U_MAX = 300.0f;   // 输出限幅（±µs）
  const float TAU   = 0.02f;    // D 项低通时间常数(s)

  // 1) dt（秒）——每轴独立计时
  uint32_t now = micros();
  float dt = (last_us[i] == 0) ? 0.02f : (now - last_us[i]) / 1e6f;
  if (dt < 0.001f) dt = 0.001f;   // ≥1ms
  if (dt > 0.05f)  dt = 0.02f;    // 卡顿保护 20ms
  last_us[i] = now;

  // 2) 误差：e = 0 - angle
  float e = -angle_deg;

  // 3) 积分（防风up：跟随输出限幅）
  I[i] += Ki * e * dt;
  if (I[i] >  U_MAX) I[i] =  U_MAX;
  if (I[i] < -U_MAX) I[i] = -U_MAX;

  // 4) 微分：对“测量”取并低通（更抗噪）
  float dmeas = (angle_deg - prev[i]) / dt; // deg/s
  prev[i] = angle_deg;
  float Draw = -Kd * dmeas;                 // 对测量取微分 → 取负号
  float alpha = dt / (TAU + dt);
  Df[i] += alpha * (Draw - Df[i]);

  // 5) 合成输出并限幅（单位：µs 的增量）
  float u = Kp * e + I[i] + Df[i];
  if (u >  U_MAX) u =  U_MAX;
  if (u < -U_MAX) u = -U_MAX;
  return u;
}



//**********************************************************************************************************************************************************************


/************ 你来写控制算法：读取输入 → 写 g_m1..g_m4 ************/
// 在 userMix() 之前放这四行，按实际旋向设置：CCW=+1，CW=-1
// 例：若 M1(前左)=CCW，则 +1；若 M2(前右)=CW，则 -1
#define YAW_M1_SIGN  (+1)
#define YAW_M2_SIGN  (-1)
#define YAW_M3_SIGN  (+1)
#define YAW_M4_SIGN  (-1)

void userMix() {
  // 基准油门
  int T = clampi(g_thr_cmd, 0, THR_MAX_US);

  // —— 如发现方向反了，把下面的 SIGN 改为 -1 即可 —— //
  const float R_SIGN = +1.0f;  // 翻滚增量方向：+1 正常；-1 反向
  const float P_SIGN = +1.0f;  // 俯仰增量方向：+1 正常；-1 反向

  // PID 增量（先稳住：降P/关I/提D，并再乘0.3柔化）
  float uR = 0.3f * R_SIGN * pidAngleDelta2(g_roll_deg,  3.0f, 0.0f, 14.0f, 0);
  float uP = 0.3f * P_SIGN * pidAngleDelta2(g_pitch_deg, 3.0f, 0.0f, 14.0f, 1);

  // 先关闭偏航混控（稳住后再开）
  float uY = 0.0f;
  // 如果要尝试偏航，改成：
  // float uY = 0.2f * pidAngleDelta2(g_yaw_deg, 6.0f, 0.0f, 8.0f, 2);

  // Quad-X 混控（1=前左 FL, 2=前右 FR, 3=后右 RR, 4=后左 RL）
  // Pitch: 前 +uP / 后 -uP
  // Roll : 右 +uR / 左 -uR
  int m1 = T + (int)(-uR - uP + uY); // 前左
  int m2 = T + (int)(+uR - uP - uY); // 前右
  int m3 = T + (int)(+uR + uP + uY); // 后右
  int m4 = T + (int)(-uR + uP - uY); // 后左

  // 允许 0=无脉冲，否则夹到有效范围
  g_m1 = (m1<=0)?0:clampi(m1, THR_MIN_US, THR_MAX_US);
  g_m2 = (m2<=0)?0:clampi(m2, THR_MIN_US, THR_MAX_US);
  g_m3 = (m3<=0)?0:clampi(m3, THR_MIN_US, THR_MAX_US);
  g_m4 = (m4<=0)?0:clampi(m4, THR_MIN_US, THR_MAX_US);

  // 简洁调试（20Hz）
  static uint32_t td=0;
  if (millis()-td >= 50) {
    td = millis();
    Serial.printf("uR=%.0f uP=%.0f uY=%.0f | T=%d M=[%d,%d,%d,%d]\n",
                  uR, uP, uY, T, g_m1,g_m2,g_m3,g_m4);
  }
}



/************ UDP 解析：写入全局变量 ************/
WiFiUDP udp;

static bool parseCSV(const String& s, int& thr, int& r, int& p, int& y){
  int c1 = s.indexOf(','); if (c1 < 0) return false;
  int c2 = s.indexOf(',', c1+1); if (c2 < 0) return false;
  int c3 = s.indexOf(',', c2+1); if (c3 < 0) return false;
  thr = s.substring(0,c1).toInt();
  r   = s.substring(c1+1,c2).toInt();
  p   = s.substring(c2+1,c3).toInt();
  y   = s.substring(c3+1).toInt();
  return true;
}
static void parseKeywords(const String& s){
  g_cmd_up=g_cmd_down=g_cmd_left=g_cmd_right=g_cmd_forward=g_cmd_back=0;
  String tok; tok.reserve(16);
  auto apply=[&](String t){
    t.trim(); if(!t.length()) return; t.toUpperCase();
    int eq=t.indexOf('='); String k=(eq>=0)?t.substring(0,eq):t; int v=(eq>=0)?t.substring(eq+1).toInt():1;
    if      (k=="T")        g_thr_cmd = clampi(v, 0, THR_MAX_US); // 允许 0
    else if (k=="UP")       g_cmd_up = v;
    else if (k=="DOWN")     g_cmd_down = v;
    else if (k=="LEFT")     g_cmd_left = v;
    else if (k=="RIGHT")    g_cmd_right = v;
    else if (k=="FORWARD")  g_cmd_forward = v;
    else if (k=="BACK")     g_cmd_back = v;
  };
  for (size_t i=0;i<s.length();++i){ char c=s[i];
    if (isalnum(c)||c=='='||c=='-') tok+=c; else {apply(tok); tok="";}
  } apply(tok);
}

/************ 电机输出管理：支持“渐停到 0 / 渐起从 0” ************/
struct Motor {
  Servo s;
  int   pin;
  bool  attached=false;
  int   cur_us=0;          // 当前真实输出（0 表示无脉冲）
  // 斜坡/复起参数
  const int ATT_MIN=500, ATT_MAX=2500; // 扩大 attach 范围，避免库夹死
  const int ESC_MIN=THR_MIN_US;        // 以你设的 THR_MIN_US 为最小有效脉宽
  const int RAMP_US_PER_MS=6;          // 斜坡速率（µs/ms）
  const int ARM_SETTLE_MS=300;         // 复起时维持 ESC_MIN 的时间
  uint32_t arm_until_ms=0;             // 非阻塞“上电臂定”计时

  void begin(int gpio){
    pin=gpio;
    s.setPeriodHertz(400);
    s.attach(pin, ATT_MIN, ATT_MAX);
    attached=true;
    cur_us=ESC_MIN;
    s.writeMicroseconds(cur_us);
  }

  // 目标 us：0 = 请求无脉冲（真 0），>0 = 正常脉宽
  void driveTo(int target_us){
    uint32_t now = millis();

    if (target_us <= 0) {
      // 目标为 0：平滑减到 ~800，然后 detach → 真无脉冲
      if (attached) {
        int next = cur_us - RAMP_US_PER_MS;
        if (next < 800) next = 800;
        cur_us = next;
        s.writeMicroseconds(cur_us);
        if (cur_us <= 800){ s.detach(); attached=false; cur_us=0; }
      } else {
        cur_us = 0;
      }
      return;
    }

    // 目标 >0：如需复起，先 attach 并在 ARM_SETTLE_MS 内维持最小油门
    if (!attached) {
      s.attach(pin, ATT_MIN, ATT_MAX);
      attached=true;
      cur_us = ESC_MIN;
      s.writeMicroseconds(cur_us);
      arm_until_ms = now + ARM_SETTLE_MS;
      return;
    }
    if (now < arm_until_ms) {
      if (cur_us != ESC_MIN) { cur_us = ESC_MIN; s.writeMicroseconds(cur_us); }
      return;
    }

    // 正常斜坡到目标
    target_us = clampi(target_us, ESC_MIN, THR_MAX_US);
    if (cur_us < target_us) {
      cur_us += RAMP_US_PER_MS;
      if (cur_us > target_us) cur_us = target_us;
      s.writeMicroseconds(cur_us);
    } else if (cur_us > target_us) {
      cur_us -= RAMP_US_PER_MS;
      if (cur_us < target_us) cur_us = target_us;
      s.writeMicroseconds(cur_us);
    }
  }
};

Motor M1, M2, M3, M4;

/************ MPU I2C 工具 ************/
static uint8_t i2cRead8(uint8_t dev, uint8_t reg){
  Wire.beginTransmission(dev); Wire.write(reg);
  if (Wire.endTransmission(false)!=0) return 0xFF;
  if (Wire.requestFrom((int)dev,1)!=1) return 0xFF;
  return Wire.read();
}
static bool i2cWrite8(uint8_t dev, uint8_t reg, uint8_t val){
  Wire.beginTransmission(dev); Wire.write(reg); Wire.write(val);
  return Wire.endTransmission()==0;
}
static bool mpuReadRaw(int16_t &ax,int16_t &ay,int16_t &az,int16_t &gx,int16_t &gy,int16_t &gz){
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B);
  if (Wire.endTransmission(false)!=0) return false;
  if (Wire.requestFrom((int)MPU_ADDR,14)!=14) return false;
  auto rd=[](){int h=Wire.read(),l=Wire.read();return (int16_t)((h<<8)|l);};
  ax=rd(); ay=rd(); az=rd(); (void)rd(); gx=rd(); gy=rd(); gz=rd();
  return true;
}
static bool mpuInit(){
  // 探测 0x68/0x69
  g_mpu_ok = false;
  for (uint8_t a: { (uint8_t)0x68, (uint8_t)0x69 }) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission()==0) { MPU_ADDR=a; g_mpu_ok=true; break; }
  }
  uint8_t who = i2cRead8(MPU_ADDR, 0x75);
  bool ok = g_mpu_ok;
  ok &= i2cWrite8(MPU_ADDR, 0x6B, 0x00); // 唤醒
  ok &= i2cWrite8(MPU_ADDR, 0x1B, 0x00); // 陀螺 ±250 dps
  ok &= i2cWrite8(MPU_ADDR, 0x1C, 0x00); // 加计 ±2 g
  ok &= i2cWrite8(MPU_ADDR, 0x1A, 0x03); // DLPF ~42 Hz
  pinMode(INT_PIN, INPUT_PULLUP);
  g_mpu_ok = ok && (who != 0xFF);
  Serial.printf("[MPU] addr=0x%02X WHO_AM_I=0x%02X init=%s\n",
                MPU_ADDR, who, g_mpu_ok?"OK":"FAIL");
  return g_mpu_ok;
}

/************ SETUP ************/
void setup() {
  Serial.begin(115200);
  delay(100);

  // 电机（平滑层）
  M1.begin(M1_PIN);
  M2.begin(M2_PIN);
  M3.begin(M3_PIN);
  M4.begin(M4_PIN);

  // Wi-Fi
  WiFi.persistent(false);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS, WIFI_CH, false, 4);
  udp.begin(UDP_PORT);

  // I2C + MPU
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  Wire.setTimeOut(50);
  Wire.begin(SDA_PIN, SCL_PIN, I2C_SPEED);
  delay(10);
  mpuInit();
  last_att_us = micros();

  uint8_t who = i2cRead8(MPU_ADDR, 0x75);
  Serial.printf("[BOOT] WHO_AM_I=0x%02X (0xFF=未连)。姿态全局变量可用：g_roll/pitch/yaw。\n", who);
}

static bool parseCSV(const String& s, int& thr, int& r, int& p, int& y);
static void parseKeywords(const String& s);

/************ LOOP ************/
void loop() {
  // ===== 收包（与你之前一致） =====
  int pkt = udp.parsePacket();
  if (pkt > 0) {
    String line = udp.readStringUntil('\n');

    if (line == "MAX") {
      g_m1=g_m2=g_m3=g_m4=THR_MAX_US;
    } else {
      int thr, r, p, y;
      if (parseCSV(line, thr, r, p, y)) {
        g_thr_cmd = clampi(thr, 0, THR_MAX_US);
      } else {
        parseKeywords(line);
        g_thr_cmd = clampi(g_thr_cmd, 0, THR_MAX_US);
      }
    }
  }

  // ===== 姿态更新（互补滤波）——不干扰你的电机逻辑 =====
  uint32_t now = micros();
  float dt = (now - last_att_us) / 1e6f;
  if (dt <= 0 || dt > 0.02f) dt = 0.002f; // 防极端（目标~500Hz）
  last_att_us = now;

  int16_t ax,ay,az,gx,gy,gz;
  bool ok = mpuReadRaw(ax,ay,az,gx,gy,gz);
  if (ok) {
    float axg = ax / ACC_SENS, ayg = ay / ACC_SENS, azg = az / ACC_SENS;
    float gx_dps = gx / GYRO_SENS, gy_dps = gy / GYRO_SENS, gz_dps = gz / GYRO_SENS;

    // 陀螺积分
    g_roll_deg  += gx_dps * dt;
    g_pitch_deg += gy_dps * dt;
    g_yaw_deg   += gz_dps * dt;  // 无磁力计，会慢漂

    // 加计角
    float roll_acc  = atan2f(ayg, azg) * 57.29578f;
    float pitch_acc = atan2f(-axg, sqrtf(ayg*ayg + azg*azg)) * 57.29578f;

    // 互补滤波融合（仅 R/P）
    g_roll_deg  = ALPHA * g_roll_deg  + (1.0f - ALPHA) * roll_acc;
    g_pitch_deg = ALPHA * g_pitch_deg + (1.0f - ALPHA) * pitch_acc;

    g_mpu_ok = true;
  } else {
    // 读取失败：保留上次角度，标记状态
    g_mpu_ok = false;
  }

  // ===== 你的算法：把 g_m1..g_m4 设为“目标”（0 = 无脉冲） =====
  userMix();

  // ===== 平滑输出到 ESC（支持 0 以及从 0 复起） =====
  M1.driveTo(g_m1);
  M2.driveTo(g_m2);
  M3.driveTo(g_m3);
  M4.driveTo(g_m4);

  // ===== 串口调试输出（每 ~50ms 一次，含姿态与电机） =====
  static uint32_t t_dbg=0;
  if (millis()-t_dbg >= 50) {
    t_dbg = millis();
    Serial.printf("MPU=%s  R=%.2f  P=%.2f  Y=%.2f  |  T=%d  M=[%d,%d,%d,%d]  OUT=[%d,%d,%d,%d]\n",
      g_mpu_ok?"OK":"NO", g_roll_deg, g_pitch_deg, g_yaw_deg,
      g_thr_cmd, g_m1,g_m2,g_m3,g_m4, M1.cur_us,M2.cur_us,M3.cur_us,M4.cur_us);
  }

  delay(1);
}
