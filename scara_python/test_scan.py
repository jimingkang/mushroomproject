import math
import numpy as np
from rplidar import RPLidar
import matplotlib.pyplot as plt
import threading
import pandas as pd
import time

from HitbotInterface import HitbotInterface
stop_event = threading.Event()
hi = HitbotInterface(92);  # //92 is robotid? yes
hi.net_port_initial()
ret = hi.initial(1, 210);  # // I add you on wechat
hi.is_connect()
hi.is_collision()
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(PORT_NAME,baudrate="460800")

# 分层参数
LAYER_COUNT = 2           # 采集 2 层
Z_STEP = 50             # 每层升高 50mm
BASE_Z = 0.0              # 起始高度 0  (最高点)

all_points = []  # 存储 (x, y, z)





def move():

    try:
        for layer in range(LAYER_COUNT):
            
            z = BASE_Z - layer * Z_STEP
            ret=hi.movel_xyz(600,0,z,0,50)

            hi.wait_stop()
            print(f"📡 正在采集第 {layer+1}/{LAYER_COUNT} 层 (高度={z:.2f}m) ...")

            # 采集 1 秒数据
            for i, scan in enumerate(lidar.iter_scans(max_buf_meas=500)):
                if i > 1:  # 只取第二帧
                    break
                for (_, angle, distance) in scan:
                    if distance == 0:
                        continue
                    r = distance / 1000.0  # mm -> m
                    x = r * math.cos(math.radians(angle))
                    y = r * math.sin(math.radians(angle))
                    all_points.append((round(x, 3), round(y, 3), round(z, 3)))

        print("✅ 所有层扫描完毕")

    finally:
        lidar.stop()
        lidar.disconnect()
   


if __name__ == '__main__':
    #watcher_thread = threading.Thread(target=printstatus)
    move_thread = threading.Thread(target=move)
    move_thread.start()
    #watcher_thread.start()
    move_thread.join()
    #watcher_thread.join()
    
    # ----------------------------
    # 分层分析：统计每个 (x, y) 的高度分布
    # ----------------------------
    points = np.array(all_points).reshape(-1, 3)


# ✅ 安全检查
    if points.ndim < 2 or len(points) == 0:
        print("⚠️ 没有采集到任何激光点，无法分类。")
        exit
    xy = np.round(points[:, :2], 2)  # 按 1cm 分辨率聚合
    z = points[:, 2]

    # 聚合统计
    xy_unique, indices = np.unique(xy, axis=0, return_inverse=True)
    z_layers = [set() for _ in range(len(xy_unique))]
    for i, idx in enumerate(indices):
        z_layers[idx].add(round(z[i], 2))

    # 分类
    pillar_points = []
    beam_points = []
    for i, xy_pt in enumerate(xy_unique):
        layer_count = len(z_layers[i])
        if layer_count >= 2:   # 在3层以上出现：立柱
            pillar_points.append(xy_pt)
        elif layer_count > 0:  # 只在1~2层：横梁
            beam_points.append(xy_pt)

    pillar_points = np.array(pillar_points)
    beam_points = np.array(beam_points)
    # ----------------------------
    # 可视化
    # ----------------------------
    plt.figure(figsize=(6, 6))
    if len(pillar_points) > 0:
        plt.scatter(pillar_points[:, 0], pillar_points[:, 1], c='r', label='Pillar (vertical)')
    if len(beam_points) > 0:
        plt.scatter(beam_points[:, 0], beam_points[:, 1], c='b', label='Beam (horizontal)')
    plt.legend()
    plt.title("LIDAR Layered Scan Classification")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.axis("equal")
    plt.show()
        
