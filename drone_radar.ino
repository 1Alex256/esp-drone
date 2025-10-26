#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ======== SoftAP 热点配置 ========
const char* AP_SSID = "ESP32-TOF-MMW";
const char* AP_PASS = "12345678";   // 至少8位
const IPAddress AP_IP(192,168,4,1);
const IPAddress AP_GW(192,168,4,1);
const IPAddress AP_MASK(255,255,255,0);
const IPAddress BCAST_IP(192,168,4,255);  // 广播地址
// =================================

// ======== 端口定义（与 Python 对齐） ========
const uint16_t TOF_UDP_PORT = 5005;    // TOF 端口
const uint16_t MMW_UDP_PORT = 5006;    // 毫米波 端口
// ==========================================

// ======== 硬件串口与波特率 ========
// TOF: UART2 -> GPIO16/17 @115200
const int TOF_RX = 16;
const int TOF_TX = 17;
const uint32_t TOF_BAUD = 115200;

// 毫米波: UART1 -> GPIO26/27 @3000000
const int MMW_RX = 26;
const int MMW_TX = 27;
const uint32_t MMW_BAUD = 3000000;
// =================================

WiFiUDP udp;

// ========= TOF（NLink 4x4）解析 =========
const uint8_t FRAME_HEADER  = 0x57;
const uint8_t FUNC_MARK     = 0x01;
const int     FRAME_LEN_4x4 = 112;
const uint8_t ZONE_4x4      = 0x10;

static uint8_t tof_buf[256];
static int     tof_blen = 0;

bool checksum_ok(const uint8_t* frame, int len) {
  uint32_t s = 0;
  for (int i = 0; i < len - 1; ++i) s += frame[i];
  return ((s & 0xFF) == frame[len - 1]);
}

// 与你之前“正确版本”一致：三字节组成 raw，再换成毫米
static bool parse_frame_4x4_to_mm(const uint8_t* f, uint16_t out_mm[16]) {
  if (f[0] != FRAME_HEADER || f[1] != FUNC_MARK) return false;
  if (f[8] != ZONE_4x4) return false;

  for (int k = 0; k < 16; ++k) {
    int base = 9 + k * 6;
    uint8_t b9  = f[base + 0];
    uint8_t b10 = f[base + 1];
    uint8_t b11 = f[base + 2];
    uint8_t st  = f[base + 3];  // 0 = 有效

    uint32_t raw = ((uint32_t)b11 << 24) | ((uint32_t)b10 << 16) | ((uint32_t)b9 << 8);
    uint32_t mm  = (raw + 128000) / 256000; // 四舍五入到 mm

    if (st == 0 && mm > 0 && mm < 65535) out_mm[k] = (uint16_t)mm;
    else out_mm[k] = 0;
  }
  return true;
}

static void send_udp_csv16(const uint16_t d[16], uint16_t port) {
  // 以 CSV16 发送到广播
  String csv;
  csv.reserve(16*6);
  for (int i=0;i<16;++i){ if(i) csv+=','; csv += String(d[i]); }
  csv += '\n';
  udp.beginPacket(BCAST_IP, port);
  udp.write((const uint8_t*)csv.c_str(), csv.length());
  udp.endPacket();
}

static void print16_line(const char* tag, const uint16_t d[16]) {
  Serial.print(tag);
  for (int i=0;i<16;++i){ if(i) Serial.print(','); Serial.print(d[i]); }
  Serial.println();
}

// ========= 毫米波帧解析（提取“距离”，填充成 16 个相同值） =========
// 你之前给的雷达点云协议：以 0xCB 0xFE 0xDD 0xFF 为包头、0xC4 0xFE 0xDD 0xFF 为包尾；
// 每点项 20B，其中 offset+4 为距离(LE32)。这里继续用“最小距离”模式。
static const uint8_t MMW_HEAD[4] = {0xCB,0xFE,0xDD,0xFF};
static const uint8_t MMW_TAIL[4] = {0xC4,0xFE,0xDD,0xFF};
enum MState { M_SEEK_HEAD, M_COLLECT };
static MState mstate = M_SEEK_HEAD;

static const size_t MMW_MAX_BODY = 8192;
static uint8_t mmw_body[MMW_MAX_BODY];
static size_t  mmw_blen = 0;
static uint8_t mwin4[4]; static int mfill=0;

inline uint32_t rd_le32(const uint8_t* p){
  return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24);
}

static bool mmw_distance_updated = false;
static uint32_t mmw_last_mm = 0;        // 最近一次计算出的距离（mm）
static uint16_t mmw_last_16[16] = {0};  // 作为 16 点共用距离的数组

static void mmw_reset(){ mstate=M_SEEK_HEAD; mmw_blen=0; mfill=0; }

static void mmw_extract_distance() {
  if (mmw_blen < 4) return;
  size_t dataLen = mmw_blen - 4;
  if (dataLen % 20 != 0) return;
  uint32_t N = dataLen / 20;
  if (N == 0) return;

  const uint8_t* p = mmw_body + 4;

  uint32_t first = rd_le32(p + 4);
  uint32_t mn = first;
  uint64_t sum = 0;

  for (uint32_t i=0;i<N;++i){
    const uint8_t* q = p + i*20;
    uint32_t dist = rd_le32(q + 4);
    if (dist < mn) mn = dist;
    sum += dist;
  }
  // 策略：用最小距离（可换平均/最大）
  uint32_t chosen = mn;

  // 这里假设毫米波“dist”的单位就是 mm；若需要换算，可在此处乘/除
  mmw_last_mm = chosen;
  for (int i=0;i<16;++i) mmw_last_16[i] = (uint16_t)chosen;
  mmw_distance_updated = true;
}

static void mmw_feed(uint8_t b){
  switch(mstate){
    case M_SEEK_HEAD:{
      if (mfill<4) mwin4[mfill++]=b; else { mwin4[0]=mwin4[1]; mwin4[1]=mwin4[2]; mwin4[2]=mwin4[3]; mwin4[3]=b; }
      if (mfill==4 && mwin4[0]==MMW_HEAD[0] && mwin4[1]==MMW_HEAD[1] && mwin4[2]==MMW_HEAD[2] && mwin4[3]==MMW_HEAD[3]) {
        mstate=M_COLLECT; mmw_blen=0; mfill=0;
      }
    } break;
    case M_COLLECT:{
      if (mmw_blen < MMW_MAX_BODY) mmw_body[mmw_blen++]=b; else { mmw_reset(); return; }
      if (mfill<4) mwin4[mfill++]=b; else { mwin4[0]=mwin4[1]; mwin4[1]=mwin4[2]; mwin4[2]=mwin4[3]; mwin4[3]=b; }
      if (mfill==4 && mwin4[0]==MMW_TAIL[0] && mwin4[1]==MMW_TAIL[1] && mwin4[2]==MMW_TAIL[2] && mwin4[3]==MMW_TAIL[3]) {
        if (mmw_blen>=4) mmw_blen-=4;  // 去尾
        mmw_extract_distance();
        mmw_reset();
      }
    } break;
  }
}

// ======== 初始化 ========
void setup(){
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[BOOT] ESP32 SoftAP + TOF@115200 + MMW@3000000 + UDP Broadcast");

  // 启动热点
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
  bool apok = WiFi.softAP(AP_SSID, AP_PASS, 1 /*channel*/, 0 /*not hidden*/, 4 /*max conn*/);
  Serial.printf("[AP] %s  SSID=%s  PASS=%s  IP=%s\n",
                apok?"OK":"FAIL", AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());
  udp.begin(0);  // 仅发送即可

  // 串口缓存放大
  Serial2.setRxBufferSize(4096);   // TOF
  Serial1.setRxBufferSize(16384);  // MMW 高速

  // 打开两路 UART
  Serial2.begin(TOF_BAUD, SERIAL_8N1, TOF_RX, TOF_TX);
  Serial1.begin(MMW_BAUD, SERIAL_8N1, MMW_RX, MMW_TX);

  Serial.printf("[TOF] UART2 @%lu, RX=%d TX=%d\n", (unsigned long)TOF_BAUD, TOF_RX, TOF_TX);
  Serial.printf("[MMW] UART1 @%lu, RX=%d TX=%d\n", (unsigned long)MMW_BAUD, MMW_RX, MMW_TX);
  Serial.println("[HINT] 电脑连上热点后，Python 脚本监听 5005/5006 即可接收（注意放行防火墙）");
}

// ======== 主循环 ========
void loop(){
  // ===== 读 TOF，提帧、校验、解析、上报 =====
  while (Serial2.available()){
    if (tof_blen < (int)sizeof(tof_buf)) tof_buf[tof_blen++] = (uint8_t)Serial2.read();
    else { memmove(tof_buf, tof_buf+1, sizeof(tof_buf)-1); tof_buf[sizeof(tof_buf)-1]=(uint8_t)Serial2.read(); tof_blen=sizeof(tof_buf); }
  }
  bool made = true;
  while (made){
    made = false;
    int start=-1;
    for (int i=0;i+1<tof_blen;++i){ if (tof_buf[i]==FRAME_HEADER && tof_buf[i+1]==FUNC_MARK){ start=i; break; } }
    if (start<0) break;
    if (start>0){ memmove(tof_buf, tof_buf+start, tof_blen-start); tof_blen-=start; }
    if (tof_blen < FRAME_LEN_4x4) break;

    uint8_t frame[FRAME_LEN_4x4];
    memcpy(frame, tof_buf, FRAME_LEN_4x4);

    if (!checksum_ok(frame, FRAME_LEN_4x4)){
      memmove(tof_buf, tof_buf+1, tof_blen-1); tof_blen-=1; continue;
    }

    uint16_t dmm[16];
    if (parse_frame_4x4_to_mm(frame, dmm)){
      Serial.print("tof测距："); print16_line("", dmm); // 串口打印
      send_udp_csv16(dmm, TOF_UDP_PORT);               // UDP 广播
    }
    memmove(tof_buf, tof_buf+FRAME_LEN_4x4, tof_blen-FRAME_LEN_4x4);
    tof_blen -= FRAME_LEN_4x4;
    made = true;
  }

  // ===== 读 毫米波，解包到一个距离，复制成16个点 =====
  while (Serial1.available()){
    uint8_t b = (uint8_t)Serial1.read();
    mmw_feed(b);
  }
  if (mmw_distance_updated){
    mmw_distance_updated = false;

    // 把 16 个同距离发出去 & 打印
    Serial.print("毫米波雷达测距："); print16_line("", mmw_last_16);
    send_udp_csv16(mmw_last_16, MMW_UDP_PORT);
  }

  // 小憩
  delay(1);
}
