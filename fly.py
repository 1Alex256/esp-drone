# File: quad_keyboard_udp_win.py
import socket, time, msvcrt

ESP32_IP   = "192.168.4.1"
ESP32_PORT = 14550

THR_MIN = 500
THR_MAX = 2250
AXIS_LIMIT = 500

STEP_THR  = 50
STEP_AXIS = 50
SEND_HZ   = 50

def clamp(x, lo, hi): return lo if x < lo else hi if x > hi else x

def main():
    thr = THR_MIN
    roll = pitch = yaw = 0

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.0)

    print("[Controls] Q/E throttle  W/S pitch  A/D roll  J/L yaw  P kill  Space zero RPY  U full  M MAX(cmd)  ESC quit")
    t_next = time.time()

    while True:
        # 非阻塞键盘
        while msvcrt.kbhit():
            ch = msvcrt.getch()
            if ch == b'\x1b':  # ESC
                print("\nBye.")
                return
            c = ch.decode('utf-8', errors='ignore').lower()
            if   c == 'q': thr   = clamp(thr + STEP_THR, THR_MIN, THR_MAX)
            elif c == 'e': thr   = clamp(thr - STEP_THR, THR_MIN, THR_MAX)
            elif c == 'w': pitch = clamp(pitch + STEP_AXIS, -AXIS_LIMIT, AXIS_LIMIT)
            elif c == 's': pitch = clamp(pitch - STEP_AXIS, -AXIS_LIMIT, AXIS_LIMIT)
            elif c == 'a': roll  = clamp(roll  - STEP_AXIS, -AXIS_LIMIT, AXIS_LIMIT)
            elif c == 'd': roll  = clamp(roll  + STEP_AXIS, -AXIS_LIMIT, AXIS_LIMIT)
            elif c == 'j': yaw   = clamp(yaw   - STEP_AXIS, -AXIS_LIMIT, AXIS_LIMIT)
            elif c == 'l': yaw   = clamp(yaw   + STEP_AXIS, -AXIS_LIMIT, AXIS_LIMIT)
            elif c == 'p': thr, roll, pitch, yaw = THR_MIN, 0, 0, 0
            elif ch == b' ':     roll = pitch = yaw = 0
            elif c == 'u': thr   = THR_MAX
            elif c == 'm':       sock.sendto(b"MAX\n", (ESP32_IP, ESP32_PORT))  # 直送满油（固件级）

            print(f"T={thr}  R={roll:+}  P={pitch:+}  Y={yaw:+}      ", end='\r')

        # 固定频率发送： "thr,roll,pitch,yaw\n"
        now = time.time()
        if now >= t_next:
            msg = f"{thr},{roll},{pitch},{yaw}\n".encode('utf-8')
            sock.sendto(msg, (ESP32_IP, ESP32_PORT))
            t_next = now + 1.0 / SEND_HZ

        time.sleep(0.001)

if __name__ == "__main__":
    main()
