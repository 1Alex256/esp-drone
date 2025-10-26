// ================= ESP32 纯透传波特率转换器 =================
// PC(USB串口) @115200  <=>  ESP32  <=>  传感器(Serial2) @3000000
// 功能：把 PC 的数据原样发给传感器；把传感器的数据原样发给 PC。
// 不添加/删除/修改任何字节；不打印任何调试文本（防止干扰PC侧流）。
#include <Arduino.h>

// ===== 可调参数 =====
static const int RX2_PIN = 26;                  // 传感器 TX -> ESP32 RX2
static const int TX2_PIN = 27;                  // 传感器 RX <- ESP32 TX2
static const unsigned long PC_BAUD   = 115200;  // 电脑串口监视器波特率（需要更快可改 921600）
static const unsigned long SENS_BAUD = 3000000; // 传感器侧波特率（3 Mbps）

// 传感器->电脑方向环形缓冲（吸收 3Mbps 突发；过大占RAM，过小易丢）
static const size_t RB_SIZE     = 32768;        // 32 KB
// 批量搬运临时缓冲
static const size_t PC2SENS_TMP = 512;
static const size_t SENS2PC_TMP = 512;

// ===== 全局缓冲与状态 =====
static uint8_t ringbuf[RB_SIZE];
static volatile size_t rb_head = 0;  // 写指针
static volatile size_t rb_tail = 0;  // 读指针

static uint8_t tmpPC2SENS[PC2SENS_TMP];
static uint8_t tmpSENS2PC[SENS2PC_TMP];

// ===== 环形缓冲操作（满了就丢旧数据以保证实时性） =====
inline void rb_push(uint8_t b) {
  size_t next = (rb_head + 1) % RB_SIZE;
  if (next == rb_tail) {             // 满：丢弃最旧一个
    rb_tail = (rb_tail + 1) % RB_SIZE;
  }
  ringbuf[rb_head] = b;
  rb_head = next;
}
inline size_t rb_pop(uint8_t* out, size_t maxlen) {
  size_t n = 0;
  while (n < maxlen && rb_tail != rb_head) {
    out[n++] = ringbuf[rb_tail];
    rb_tail = (rb_tail + 1) % RB_SIZE;
  }
  return n;
}

// ===== 初始化 =====
void setup() {
  // 放大底层接收缓冲以减少溢出
  Serial.setRxBufferSize(4096);
  Serial2.setRxBufferSize(16384);

  Serial.begin(PC_BAUD);
  Serial2.begin(SENS_BAUD, SERIAL_8N1, RX2_PIN, TX2_PIN);

  // 不打印任何内容，避免影响 PC 通道
}

// 从 PC 读 -> 发给传感器
void pump_PC_to_Sensor() {
  int avail = Serial.available();
  while (avail > 0) {
    int chunk = avail > (int)sizeof(tmpPC2SENS) ? (int)sizeof(tmpPC2SENS) : avail;
    int got = Serial.readBytes(tmpPC2SENS, chunk);
    if (got > 0) {
      Serial2.write(tmpPC2SENS, got);
      // 不必 flush，每轮 loop 频繁调用即可
    }
    avail = Serial.available();
  }
}

// 从传感器读 -> 先入环形缓冲
void pump_Sensor_to_PC_into_ring() {
  int avail = Serial2.available();
  while (avail > 0) {
    int chunk = avail > (int)sizeof(tmpSENS2PC) ? (int)sizeof(tmpSENS2PC) : avail;
    int got = Serial2.readBytes(tmpSENS2PC, chunk);
    for (int i = 0; i < got; ++i) rb_push(tmpSENS2PC[i]);
    avail = Serial2.available();
  }
}

// 从环形缓冲 -> 吐给 PC（受 PC 115200 限制，慢慢排空）
void drain_ring_to_PC() {
  int space = Serial.availableForWrite();
  if (space <= 0) return;
  int toSend = min(space, (int)sizeof(tmpSENS2PC));
  size_t n = rb_pop(tmpSENS2PC, toSend);
  if (n > 0) {
    Serial.write(tmpSENS2PC, n);
    // 不调用 flush，保持吞吐
  }
}

// ===== 主循环 =====
void loop() {
  pump_PC_to_Sensor();            // PC -> 传感器（115200 → 3M）
  pump_Sensor_to_PC_into_ring();  // 传感器 -> 环形缓冲
  drain_ring_to_PC();             // 环形缓冲 -> PC（3M → 115200）
}
