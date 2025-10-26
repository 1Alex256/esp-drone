# -*- coding: utf-8 -*-
# TOF 4x4 (UDP:5005, CSV16, mm) + 毫米波 4x4 (UDP:5006, CSV16, mm同距)
# 特性：
#  - 毫米波：无新数据保持上一帧
#  - 毫米波：建模前将距离对半
#  - R/r 开始录制  P/p 暂停  C 清空  S 保存  ←→↑↓/WASD 平移  F 取景
#  - 控制台打印 TOF/毫米波原始与点数；毫米波也打印“对半后用于建模”的值

import time, socket, numpy as np, open3d as o3d

UDP_BIND_IP   = "0.0.0.0"
UDP_PORT_TOF  = 5005
UDP_PORT_MMW  = 5006
EXPECT_SRC_IP = None

H, W = 4, 4
SQUARE_SIZE = 0.1  # m
POINT_SIZE_TOF = 6.0
POINT_SIZE_MMW = 9.0

def parse_csv16(line: str):
    parts = [p for p in line.replace(',', ' ').split() if p]
    if len(parts) != 16: return None
    try:
        return [float(p) for p in parts]
    except ValueError:
        return None

def grid_points_mm_as_m(d_mm, x0, y0, z_sign=+1.0, square_size=SQUARE_SIZE):
    pts = []
    idx = 0
    for r in range(H):
        for c in range(W):
            d = d_mm[idx]; idx += 1
            if d is None or d <= 0: continue
            gx = (c / (W - 1)) * square_size
            gy = (r / (H - 1)) * square_size
            z  = z_sign * (d / 1000.0)  # mm -> m
            pts.append((gx + x0, gy + y0, z))
    if pts:
        return np.asarray(pts, dtype=np.float32)
    return np.zeros((0,3), np.float32)

# 位姿/交互
pos_x = 0.0; pos_y = 0.0; STEP = 0.02
def _print_pose(): print(f"[POSE] x={pos_x:.3f} m, y={pos_y:.3f} m")
def cb_move(dx, dy):
    def _fn(vis):
        global pos_x, pos_y
        pos_x += dx; pos_y += dy; _print_pose(); return False
    return _fn

def fit_view_to_all(vis, geoms):
    valid_boxes = [g.get_axis_aligned_bounding_box() for g in geoms if len(g.points) > 0]
    if not valid_boxes:
        return False
    bbox = valid_boxes[0]
    for b in valid_boxes[1:]:
        bbox += b
    ctr = vis.get_view_control()
    center = bbox.get_center()
    extent = np.linalg.norm(bbox.get_max_bound() - bbox.get_min_bound())
    if extent < 1e-3: extent = 0.2
    ctr.set_lookat(center)
    ctr.set_front([-0.5, -0.3, -0.8])  # 斜视角：同时看 +Z/-Z
    ctr.set_up([0, 1, 0])
    ctr.set_zoom(0.6 if extent < 1.0 else 0.35)
    return True

def main():
    global pos_x, pos_y
    print(f"Listening UDP: TOF {UDP_BIND_IP}:{UDP_PORT_TOF} | MMW {UDP_BIND_IP}:{UDP_PORT_MMW}")

    s_tof = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s_tof.bind((UDP_BIND_IP, UDP_PORT_TOF)); s_tof.setblocking(False)
    s_mmw = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s_mmw.bind((UDP_BIND_IP, UDP_PORT_MMW)); s_mmw.setblocking(False)

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window("TOF+MMW Realtime (CSV16 + CSV16, hold-last MMW, halve before modeling)", 1000, 750)

    # TOF：累计（白），当前（黄）
    pcd_tof_acc = o3d.geometry.PointCloud(); pcd_tof_acc.paint_uniform_color([1,1,1]); vis.add_geometry(pcd_tof_acc)
    pcd_tof_cur = o3d.geometry.PointCloud(); pcd_tof_cur.paint_uniform_color([1.0,0.8,0.0]); vis.add_geometry(pcd_tof_cur)

    # MMW：累计/当前（固定蓝色）
    pcd_mmw_acc = o3d.geometry.PointCloud(); pcd_mmw_acc.paint_uniform_color([0.0,0.4,1.0]); vis.add_geometry(pcd_mmw_acc)
    pcd_mmw_cur = o3d.geometry.PointCloud(); pcd_mmw_cur.paint_uniform_color([0.0,0.6,1.0]); vis.add_geometry(pcd_mmw_cur)

    # 坐标轴
    axis = o3d.geometry.LineSet()
    axis.points = o3d.utility.Vector3dVector([[0,0,0],[0.2,0,0],[0,0,0],[0,0.2,0],[0,0,0],[0,0,0.2]])
    axis.lines  = o3d.utility.Vector2iVector([[0,1],[2,3],[4,5]])
    axis.colors = o3d.utility.Vector3dVector([[1,0,0],[1,0,0],[0,1,0],[0,1,0],[0,0,1],[0,0,1]])
    vis.add_geometry(axis)

    # 渲染选项
    opt = vis.get_render_option()
    opt.point_size = POINT_SIZE_TOF
    opt.background_color = np.array([0,0,0])

    # 键盘
    GLFW_LEFT, GLFW_RIGHT, GLFW_DOWN, GLFW_UP = 263, 262, 264, 265
    vis.register_key_callback(GLFW_LEFT,  cb_move(-STEP, 0.0))
    vis.register_key_callback(GLFW_RIGHT, cb_move(+STEP, 0.0))
    vis.register_key_callback(GLFW_UP,    cb_move(0.0, +STEP))
    vis.register_key_callback(GLFW_DOWN,  cb_move(0.0, -STEP))
    for k,dx,dy in [('A',-STEP,0),('a',-STEP,0),('D',+STEP,0),('d',+STEP,0),('W',0,+STEP),('w',0,+STEP),('s',0,-STEP)]:
        vis.register_key_callback(ord(k), cb_move(dx,dy))
    vis.register_key_callback(ord('F'), lambda v: (fit_view_to_all(v, [pcd_tof_cur, pcd_tof_acc, pcd_mmw_cur, pcd_mmw_acc]), False)[1])
    vis.register_key_callback(ord('f'), lambda v: (fit_view_to_all(v, [pcd_tof_cur, pcd_tof_acc, pcd_mmw_cur, pcd_mmw_acc]), False)[1])

    # 录制/保存
    state = {"recording": False}
    def start_rec(v): state["recording"]=True;  print("[R] 开始录制"); return False
    def pause_rec(v): state["recording"]=False; print("[P] 暂停录制"); return False
    def clear_all(v):
        pcd_tof_acc.points = o3d.utility.Vector3dVector(np.zeros((0,3),np.float32))
        pcd_mmw_acc.points = o3d.utility.Vector3dVector(np.zeros((0,3),np.float32))
        print("[C] 清空累计"); return False
    def save_ply(v):
        cloud = []
        if len(pcd_tof_acc.points) > 0: cloud.append(np.asarray(pcd_tof_acc.points))
        if len(pcd_mmw_acc.points) > 0: cloud.append(np.asarray(pcd_mmw_acc.points))
        if cloud:
            cloud = np.vstack(cloud).astype(np.float32)
            o3d.io.write_point_cloud("record_all.ply", o3d.geometry.PointCloud(o3d.utility.Vector3dVector(cloud)), write_ascii=True)
            print(f"[S] 已保存累计点 -> record_all.ply, 点数={cloud.shape[0]}")
        else:
            print("[S] 无累计点可保存")
        return False

    vis.register_key_callback(ord('R'), start_rec)
    vis.register_key_callback(ord('r'), start_rec)
    vis.register_key_callback(ord('P'), pause_rec)
    vis.register_key_callback(ord('p'), pause_rec)
    vis.register_key_callback(ord('C'), clear_all)
    vis.register_key_callback(ord('c'), clear_all)
    vis.register_key_callback(ord('S'), save_ply)

    # 初始相机
    fit_view_to_all(vis, [pcd_tof_cur, pcd_tof_acc, pcd_mmw_cur, pcd_mmw_acc])

    last_draw=time.time()
    last_mmw_vals = None            # 存“对半后”的 mm 值
    last_mmw_pts  = np.zeros((0,3), np.float32)
    last_mmw_ts   = 0.0

    try:
        while True:
            updated=False

            # ===== 读 TOF(5005)：CSV16 =====
            while True:
                try: data, addr = s_tof.recvfrom(4096)
                except BlockingIOError: break
                if not data: break
                if EXPECT_SRC_IP and addr[0] != EXPECT_SRC_IP: continue

                line = data.decode(errors="ignore").strip()
                vals = parse_csv16(line)
                if vals is None: continue

                print("tof测距：" + ",".join(f"{int(v)}" for v in vals))
                pts = grid_points_mm_as_m(vals, pos_x, pos_y, z_sign=+1.0)
                pcd_tof_cur.points = o3d.utility.Vector3dVector(pts)
                pcd_tof_cur.paint_uniform_color([1.0, 0.8, 0.0])
                if pts.size>0 and state["recording"]:
                    cloud = np.asarray(pcd_tof_acc.points)
                    cloud = np.vstack([cloud, pts]) if cloud.size else pts
                    pcd_tof_acc.points = o3d.utility.Vector3dVector(cloud)
                    pcd_tof_acc.paint_uniform_color([1.0, 1.0, 1.0])
                print(f"[TOF] 当前帧点数: {pts.shape[0]}")
                vis.update_geometry(pcd_tof_cur); vis.update_geometry(pcd_tof_acc)
                updated=True

            # ===== 读 MMW(5006)：CSV16（建模前对半；无新数据保持） =====
            mmw_frame_arrived = False
            while True:
                try: data, addr = s_mmw.recvfrom(4096)
                except BlockingIOError: break
                if not data: break
                if EXPECT_SRC_IP and addr[0] != EXPECT_SRC_IP: continue

                line = data.decode(errors="ignore").strip()
                vals_raw = parse_csv16(line)
                if vals_raw is None: continue

                print("毫米波雷达测距（原始）：" + ",".join(f"{int(v)}" for v in vals_raw))
                # —— 对半 ——（单位仍 mm）
                vals = [v/2.0 for v in vals_raw]

                if any(v > 0 for v in vals):
                    last_mmw_vals = vals[:]   # 存“对半后”的值
                    last_mmw_ts   = time.time()
                    mmw_frame_arrived = True
                    print("毫米波雷达测距（对半用于建模）：" + ",".join(f"{int(v)}" for v in vals))
                else:
                    print("[MMW] 收到全 0（或对半后全 0），保持上一次可见帧")

            # —— 渲染毫米波（使用 last_mmw_vals；无新包也保持）——
            if last_mmw_vals is not None:
                pts = grid_points_mm_as_m(last_mmw_vals, pos_x, pos_y, z_sign=-1.0)
                if pts.size > 0:
                    last_mmw_pts = pts
                    pcd_mmw_cur.points = o3d.utility.Vector3dVector(pts)
                    pcd_mmw_cur.paint_uniform_color([0.0, 0.6, 1.0])
                    if state["recording"]:
                        cloud = np.asarray(pcd_mmw_acc.points)
                        cloud = np.vstack([cloud, pts]) if cloud.size else pts
                        pcd_mmw_acc.points = o3d.utility.Vector3dVector(cloud)
                        pcd_mmw_acc.paint_uniform_color([0.0, 0.4, 1.0])
                    print(f"[MMW] 当前帧点数: {pts.shape[0]} (age={time.time()-last_mmw_ts:.1f}s)")
                    # 临时加粗可见性
                    vis.get_render_option().point_size = POINT_SIZE_MMW
                    vis.update_geometry(pcd_mmw_cur); vis.update_geometry(pcd_mmw_acc)
                    updated=True
                else:
                    if last_mmw_pts.size > 0:
                        pcd_mmw_cur.points = o3d.utility.Vector3dVector(last_mmw_pts)
                        pcd_mmw_cur.paint_uniform_color([0.0, 0.6, 1.0])
                        vis.get_render_option().point_size = POINT_SIZE_MMW
                        vis.update_geometry(pcd_mmw_cur)
                        print("[MMW] 没有新有效点，继续显示上一帧")
                        updated=True

            if updated or (time.time()-last_draw>0.05):
                if not vis.poll_events(): break
                vis.update_renderer(); last_draw=time.time()

            time.sleep(0.002)

    finally:
        vis.destroy_window()
        s_tof.close()
        s_mmw.close()

if __name__ == "__main__":
    main()
