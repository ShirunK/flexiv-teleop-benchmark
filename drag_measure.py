#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
drag_measure_6dir.py

功能：
1) 在脚本开始时，列出当前目录可执行的 "test_" 开头文件，让用户选择一个要测试的 Teleop 可执行文件。
2) 对 (X+/X-/Y+/Y-/Z+/Z-) 六方向循环测量阻尼：
   - (a) 同步到预设Pose (关节角度)
   - (b) 启动Teleop进程 (sudo + Popen)，创建新会话
   - (c) 分段阻尼测量 (末端 5cm~finalDistM, 每5cm一段)
   - (d) 测量完毕后, 首先 killpg(进程组)，然后使用 pkill -f 强制结束剩余进程
3) 在程序中加入异常处理 & Ctrl+C (SIGINT) 捕捉，一旦出错或中断, 杀掉Teleop进程树，并通过 pkill -f 进一步清理。
4) 将测量数据与结果写到同目录下的CSV文件。
5) 此方法相对激进，但能**最大限度**清除 Teleop 及其可能外派的所有子进程。

注意：
- pkill -f 会匹配所有进程命令行中含 <pattern> 的进程并 kill -9，需谨慎使用。
- 你可将 pattern 换成更独特的可执行名，避免误杀其它进程。
"""

import os
import sys
import time
import math
import csv
import signal
import subprocess
from datetime import datetime
import argparse
import flexivrdk

# ========== 全局变量 ==========
teleop_pid = None            # Teleop 主进程PID
teleop_pattern = None        # Teleop 可执行文件名(去掉路径)，供 pkill -f

# 机器人序列号
leader_robot_sn   = None
follower_robot_sn = None

# 末端目标位移 (米)
finalDistM = 0.30
startDist  = 0.05
sample_period = 0.01

# sudo 密码
SUDO_PASSWORD = None

# =========== 各方向起始关节姿态 (单位:度) ===========
POSE_X1_START = [-0.7858643304937192, -13.877941212036967, 0.7483301998416312,
                 135.73910879757133, -0.3498534951328406, 59.615865011733526, 0.20912087569310536]
POSE_X2_START = [-0.2855396601635595, -56.758223145490426, 0.5327849576037542, 57.86894167833725, 
                 -0.48200287464114594, 24.624889557006437, 0.238431685366579]
POSE_Y1_START = [-30.054273523613276, -35.45994478096467, -5.216496361647684,
                 86.32941104939634, 3.6267676975946497, 31.560050198127808, -36.204653966689314]
POSE_Y2_START = [11.499646290703584, -15.723928755400058, 12.734033082459526,
                 119.45227683180816, -4.942539608421156, 44.72109048587742, 27.27025255890027]
POSE_Z1_START = [-3.9142230390577724, -27.200417289718366, 3.2210181754397524, 123.34058058362135,
                  -3.0135921604966938, 60.46807454482518, 1.5388601898695797]
POSE_Z2_START = [-2.165982546368497, -2.4958115276766875, 1.689129474279949,
                 72.77022195937903, -0.012274134254111531, -14.737328420174004, -0.35489540204087305]

DIRECTION_CONFIG = {
    "X+": {"vector": (1.0, 0.0, 0.0),   "jpos_start_deg": POSE_X1_START},
    "X-": {"vector": (-1.0,0.0, 0.0),  "jpos_start_deg": POSE_X2_START},
    "Y+": {"vector": (0.0, 1.0, 0.0),   "jpos_start_deg": POSE_Y1_START},
    "Y-": {"vector": (0.0,-1.0, 0.0),  "jpos_start_deg": POSE_Y2_START},
    "Z+": {"vector": (0.0, 0.0, 1.0),   "jpos_start_deg": POSE_Z1_START},
    "Z-": {"vector": (0.0, 0.0, -1.0), "jpos_start_deg": POSE_Z2_START},
}

# =========== 1) 查找可执行文件 ===========

def find_executables_in_current_dir():
    files = os.listdir('.')
    candidates = []
    for f in files:
        if os.path.isfile(f) and os.access(f, os.X_OK):
            # 只要以 test_ 开头就纳入选择
            if f.startswith("test_"):
                candidates.append((f, f"./{f}"))
    return candidates

# =========== 2) 启动 & 停止 Teleop (进程树 + pkill) ===========

def start_teleop(executable_path):
    """
    启动Teleop进程，并让它成为session leader (new session)。
    """
    global teleop_pid, teleop_pattern

    teleop_pattern = os.path.basename(executable_path)
    if "high_transparency" in teleop_pattern:
        cmd = f"echo {SUDO_PASSWORD} | sudo -S {executable_path} -l {leader_robot_sn} -r {follower_robot_sn}"
    else:
        cmd = f"echo {SUDO_PASSWORD} | sudo -S {executable_path} -1 {leader_robot_sn} -2 {follower_robot_sn}"
    pid = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid).pid
    teleop_pid = pid
    print(f"[StartTeleop] pattern=({teleop_pattern}), cmd=({cmd}), pid={pid}")
    time.sleep(2.0)
    return pid


def stop_teleop():
    """
    killpg (SIGTERM+SIGKILL) 结束该进程组
    """
    global teleop_pid, teleop_pattern

    if teleop_pid is not None and teleop_pid > 0:
        try:
            pgid = os.getpgid(teleop_pid)
        except ProcessLookupError:
            print(f"[stop_teleop] pid {teleop_pid} not found. Possibly ended.")
            pgid = None

        if pgid is not None:
            # 尝试 SIGTERM
            print(f"[stop_teleop] killpg(SIGTERM) pgid={pgid}")
            try:
                os.killpg(pgid, signal.SIGTERM)
                time.sleep(1.0)
            except ProcessLookupError:
                print("[stop_teleop] Group not found, likely ended.")
            else:
                # 检查是否活着
                try:
                    os.killpg(pgid, 0)  # 不发送实际信号，仅测试
                    print(f"[stop_teleop] Still alive => killpg(SIGKILL) pgid={pgid}")
                    os.killpg(pgid, signal.SIGKILL)
                except ProcessLookupError:
                    print("[stop_teleop] Group ended after SIGTERM.")
        teleop_pid = None

    # pkill
    if teleop_pattern:
        print(f"[stop_teleop] pkill -9 -f {teleop_pattern}")
        try:
            subprocess.run(["pkill", "-9", "-f", teleop_pattern], check=False)
        except Exception as e:
            print(f"[stop_teleop] pkill error: {e}")
    teleop_pattern = None

# =========== 同步姿态 ===========

def sync_pose(robot, jpos_deg):
    mode = flexivrdk.Mode
    robot.SwitchMode(mode.NRT_PRIMITIVE_EXECUTION)
    target = flexivrdk.JPos(jpos_deg, [0,0,0,0,0,0])
    robot.ExecutePrimitive("MoveJ", {"target": target})
    print(f"[SyncPose] Send MoveJ command to {jpos_deg}")

# =========== 分段测量阻尼 ===========

def measure_damping_in_one_direction(robot, dname, dir_vec):
    start_time = time.time()
    data_records = []
    reachedStart = False
    doneFinal    = False

    st_pose = robot.states().tcp_pose
    ix, iy, iz = st_pose[0], st_pose[1], st_pose[2]
    L = math.sqrt(dir_vec[0]**2 + dir_vec[1]**2 + dir_vec[2]**2)
    if L<1e-9:
        print(f"{dname} direction invalid.")
        return 0.0, [], []

    unit_dir = (dir_vec[0]/L, dir_vec[1]/L, dir_vec[2]/L)

    while True:
        states = robot.states()
        pose   = states.tcp_pose
        vel    = states.tcp_vel
        wrench = states.ext_wrench_in_world

        px, py, pz = pose[0], pose[1], pose[2]
        vx, vy, vz = vel[0], vel[1], vel[2]
        fx, fy_, fz_ = wrench[0], wrench[1], wrench[2]

        dx = px - ix
        dy = py - iy
        dz = pz - iz
        dist_dir = dx*unit_dir[0] + dy*unit_dir[1] + dz*unit_dir[2]
        dist_abs = abs(dist_dir)

        if not reachedStart and dist_abs >= startDist:
            print("  -> Reached 5cm, start recording data.")
            reachedStart = True

        if reachedStart and not doneFinal:
            if dist_abs <= finalDistM:
                t_now = time.time() - start_time
                data_records.append((t_now, px, py, pz, vx, vy, vz, fx, fy_, fz_, dist_abs))
            else:
                print(f"  -> Reached {int(finalDistM*100)}cm, stop recording.")
                doneFinal = True
                break

        time.sleep(sample_period)
        if doneFinal:
            break

    if len(data_records)<5:
        print(f"[Warning] {dname} data <5 => B_dir=0.")
        return 0.0, data_records, []

    # 分段: [5cm, finalDistM], step=5cm
    maxRange = finalDistM - startDist
    nChunks = int(math.floor(maxRange/0.05))
    chunkRecords = [[] for _ in range(nChunks)]

    for rec in data_records:
        dist_abs = rec[10]
        offset = dist_abs - startDist
        idx = int(offset//0.05)
        if idx<0 or idx>=nChunks:
            continue
        chunkRecords[idx].append(rec)

    chunk_result_list = []
    validSumB = 0.0
    validCount= 0

    for i in range(nChunks):
        ds = startDist + 0.05*i
        de = ds + 0.05
        samples = chunkRecords[i]
        if len(samples)==0:
            chunk_result_list.append((i, ds, de, 0.0,0.0,0.0,0))
            continue
        sumV=0.0
        sumF=0.0
        N=len(samples)
        for r in samples:
            vx, vy, vz = r[4], r[5], r[6]
            fx, fy_, fz_ = r[7], r[8], r[9]
            vdir = vx*unit_dir[0] + vy*unit_dir[1] + vz*unit_dir[2]
            fdir = fx*unit_dir[0] + fy_*unit_dir[1] + fz_*unit_dir[2]
            sumV += abs(vdir)
            sumF += abs(fdir)
        avgV = sumV/N
        avgF = sumF/N
        if avgV<1e-6:
            bc=0.0
        else:
            bc= avgF / avgV

        chunk_result_list.append((i, ds, de, avgV, avgF, bc, N))
        if bc>0 and avgV>1e-6:
            validSumB += bc
            validCount +=1

    if validCount==0:
        B_dir=0.0
    else:
        B_dir= validSumB/validCount

    # 打印分段
    print(f"  => {dname} {nChunks} chunk(s).")
    for (i, ds, de, avV, avF, bc, nm) in chunk_result_list:
        print(f"     Chunk {i}: [{ds*100:.0f}-{de*100:.0f}cm], N={nm}, V={avV:.4f}, F={avF:.4f}, B={bc:.4f}")

    print(f"  ==> {dname} overall B_dir={B_dir:.4f}\n")
    return B_dir, data_records, chunk_result_list

# ======= 异常 / Ctrl+C 处理 =======

def safe_exit():
    stop_teleop()
    sys.exit(0)

def signal_handler(sig, frame):
    print("\n[Ctrl+C] Caught, cleaning teleop & exit.")
    safe_exit()

signal.signal(signal.SIGINT, signal_handler)

# =========== 主程序 ===========

def main():
    global teleop_pid, leader_robot_sn, follower_robot_sn, SUDO_PASSWORD

    parser = argparse.ArgumentParser(description="Transparency Measurement for Master Force Feedback")
    parser.add_argument("-1", "--leader", required=True, help="主机械臂序列号")
    parser.add_argument("-2", "--follower", required=True, help="从机械臂序列号")
    parser.add_argument("-p", "--password", required=True, help="用于启动和停止 Teleop 程序的 sudo 密码")
    args = parser.parse_args()

    leader_robot_sn = args.leader
    follower_robot_sn = args.follower
    SUDO_PASSWORD = args.password
    # 1) 找test_开头可执行文件
    exe_list = find_executables_in_current_dir()
    if not exe_list:
        print("No test_ executables found in current dir.")
        sys.exit(1)

    print("可用的遥操作程序:")
    for i, (fn, fp) in enumerate(exe_list):
        print(f"  {i}: {fn}")
    choice = input("请选择要测试的程序序号: ")
    try:
        cidx = int(choice)
        if cidx<0 or cidx>=len(exe_list):
            print("无效选择.")
            sys.exit(1)
    except:
        print("输入错误.")
        sys.exit(1)

    chosen_name, exe_path = exe_list[cidx]
    print(f"\n已选择: {chosen_name}\n路径: {exe_path}\n")

    # 2) 连接Robot
    print("连接到 Robot...")
    leader_robot = flexivrdk.Robot(leader_robot_sn)
    follower_robot = flexivrdk.Robot(follower_robot_sn)
    # robot.enable()  # 如果需要

    # 打开 CSV
    now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_name = f"damping_data_{now_str}.csv"
    fcsv = open(csv_name, mode="w", newline='', encoding="utf-8")
    writer = csv.writer(fcsv)
    writer.writerow(["Damping Measurement w/ Teleop", now_str])
    writer.writerow(["finalDistM(cm)", f"{finalDistM*100:.1f}"])
    writer.writerow([])

    direction_order = ["X+", "X-", "Y+", "Y-", "Z+", "Z-"]
    results = []

    try:
        for dname in direction_order:
            dcfg = DIRECTION_CONFIG[dname]
            dvec = dcfg["vector"]
            jpos_deg = dcfg["jpos_start_deg"]

            print(f"\n========== 测量方向 {dname} ===========")

            # (a) 同步Pose
            input("[STEP]按回车键同步到起始姿态...")
            sync_pose(leader_robot, jpos_deg)
            sync_pose(follower_robot, jpos_deg)

            input(f"[STEP]位置就绪后，按回车启动teleop并开始测量 [{dname}]...")
            leader_robot.Stop()
            follower_robot.Stop()
            time.sleep(1.0)
            # (b) 启动teleop
            start_teleop(exe_path)
            time.sleep(7.0)

            # (c) 分段测量
            print(f"[STEP]现在请向{dname}方向开始移动大约 {int(finalDistM*100)}cm...")
            B_dir, data_recs, chunk_info = measure_damping_in_one_direction(leader_robot, dname, dvec)
            results.append(B_dir)

            # (d) 停止teleop
            stop_teleop()
            leader_robot.Stop()
            follower_robot.Stop()

            # 写CSV
            writer.writerow([f"Direction={dname}"])
            writer.writerow(["time_s","px","py","pz","vx","vy","vz","fx","fy","fz","dist_abs"])
            for rec in data_recs:
                writer.writerow([f"{x:.4f}" for x in rec])
            writer.writerow([])

            writer.writerow(["ChunkIndex","DistStart_m","DistEnd_m","avgV","avgF","Bchunk","N"])
            for ci in chunk_info:
                idx, ds, de, avV, avF, bc, nm = ci
                writer.writerow([idx, f"{ds:.3f}", f"{de:.3f}",
                                 f"{avV:.4f}", f"{avF:.4f}", f"{bc:.4f}", nm])
            writer.writerow(["B_dir", f"{B_dir:.4f}"])
            writer.writerow([])

        # 汇总
        valid_b = [abs(x) for x in results if x>1e-9]
        if not valid_b:
            print("\n[Warning] 所有方向均无有效数据.")
        else:
            mean_abs = sum(valid_b)/len(valid_b)
            print("\n========== 6方向阻尼结果 ===========")
            for i, dname in enumerate(direction_order):
                print(f"  {dname}: B_dir={results[i]:.4f}")
            print(f"  => 绝对值平均 = {mean_abs:.4f}")
            print("====================================\n")

            writer.writerow(["FinalResults"])
            for i, dn in enumerate(direction_order):
                writer.writerow([dn, f"{results[i]:.4f}"])
            writer.writerow(["Mean(|B_dir|)", f"{mean_abs:.4f}"])

    except Exception as e:
        print(f"[Error] 发生异常: {e}")
        safe_exit()
    finally:
        stop_teleop()
        fcsv.close()
        print(f"数据已写入 {csv_name}，程序结束。")


if __name__ == "__main__":
    main()
