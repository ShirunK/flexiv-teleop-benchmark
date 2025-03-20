#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
tracking_stiffness_measure.py

功能：
1. 同步主从机械臂到 Home Pose，并启动遥操作程序，使主手保持固定状态。  
2. 将从手旁放置固定刚体。  
3. 对于每个测试方向（例如 X, Y, Z, Rx, Ry, Rz）：  
   - 提示用户将主手移动到测试 Pose，并保持该状态；  
   - 在测试过程中，连续采样主手和从手的 TCP 位姿及外力（ext_wrench_in_world），采样频率为 100 Hz，每 0.1 秒计算一段数据；  
   - 对于平移方向，计算主从 TCP 在该轴上的位置差 Delta（单位 m）；  
     对于旋转方向，将 TCP 的四元数转换为 Euler 角后，计算对应角度差Delta（单位 rad）。  
   - 同时，采集从手在该方向上的外力F_slave；. 
   - 对每个 0.1 秒的数据段，计算局部刚度。
   - 当连续 20 个 0.1 秒内计算得到的局部刚度均相差不超过 20 (N/m 或 Nm/rad) 时，认为刚度已稳定；  
   - 在刚度稳定后，取这 20 个小段的平均值作为该次测试的跟踪刚度。  
4. 每个方向重复测试 5 次，记录每次的跟踪刚度，并计算该方向的平均刚度作为最终结果。 
"""

import argparse
import time
import signal
import sys
import csv
import os
import subprocess
from datetime import datetime
import math
import flexivrdk

teleop_pid = None
teleop_pattern = None

# 测试参数
sample_interval = 0.01   # 采样周期 (100Hz)
segment_duration = 0.1   # 每个采样段时长 (秒)
stable_count = 20        # 需要连续20个小段满足条件
min_stiffness = 10.0    # 每段刚度需大于 min_stiffness 视为有效记录
max_fluctuation = 20.0  # 连续 stable_count 段刚度最大-最小 < 20
# Home Pose（单位：度）
HOME_POSE = [-5.6850536735238373e-05, -39.999988598597405, -7.796941005345694e-05,
             89.99967467229428, -1.394160247868911e-05, 39.99993054198945, -1.9290991341998603e-06]

# 测试自由度配置：
# 对于平移方向，直接取 tcp_pose[0,1,2] 的位置差；对于旋转方向，
# 将 TCP 的四元数（索引3~6）转换为 Euler 角，再取对应角度差 (单位 rad)
# 旋转方向由于没有能使机械臂末端保持一定扭矩的固定物体，不好测量
TEST_AXES = {
    "X": {"type": "linear", "index": 0},
    "Y": {"type": "linear", "index": 1},
    "Z": {"type": "linear", "index": 2}
}
# TEST_AXES = {
#     "X": {"type": "linear", "index": 0},
#     "Y": {"type": "linear", "index": 1},
#     "Z": {"type": "linear", "index": 2},
#     "Rx": {"type": "angular", "index": 0},
#     "Ry": {"type": "angular", "index": 1},
#     "Rz": {"type": "angular", "index": 2},
# }

def quat_to_euler(q):
    # q: [qw, qx, qy, qz]
    qw, qx, qy, qz = q
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (qw * qy - qz * qx)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (roll, pitch, yaw)

def parse_args():
    parser = argparse.ArgumentParser(description="Slave Tracking Stiffness Measurement")
    parser.add_argument("-1", "--leader", required=True, help="主机械臂序列号")
    parser.add_argument("-2", "--follower", required=True, help="从机械臂序列号")
    parser.add_argument("-p", "--password", required=True, help="用于启动和停止 Teleop 程序的 sudo 密码")
    parser.add_argument("-n", "--num", type=int, default=5, help="每个方向测试次数 (默认5次)")
    return parser.parse_args()

def find_executables():
    return [(f, f"./{f}") for f in os.listdir('.') if f.startswith("test_") and os.access(f, os.X_OK)]

def start_teleop(executable_path):
    """
    启动 Teleop 程序，并使其成为新会话的组长。
    若文件名包含 "high_transparency"，则使用 -l / -r 参数，否则使用 -1 / -2 参数。
    返回 Teleop 进程的 PID。
    """
    global teleop_pid, teleop_pattern, SUDO_PASSWORD, leader_sn, follower_sn
    teleop_pattern = os.path.basename(executable_path)
    if "high_transparency" in teleop_pattern:
        cmd = f"echo {SUDO_PASSWORD} | sudo -S {executable_path} -l {leader_sn} -r {follower_sn}"
    else:
        cmd = f"echo {SUDO_PASSWORD} | sudo -S {executable_path} -1 {leader_sn} -2 {follower_sn}"
    pid = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid).pid
    teleop_pid = pid
    print(f"[StartTeleop] pattern=({teleop_pattern}), cmd=({cmd}), pid={pid}")
    time.sleep(2.0)
    return pid

def stop_teleop():
    """
    结束 Teleop 程序：先通过 killpg(SIGTERM+SIGKILL) 结束进程组，
    然后使用 pkill -9 -f 以确保所有相关进程被关闭。
    """
    global teleop_pid, teleop_pattern
    if teleop_pid is not None and teleop_pid > 0:
        try:
            pgid = os.getpgid(teleop_pid)
        except ProcessLookupError:
            print(f"[stop_teleop] pid {teleop_pid} not found. Possibly ended.")
            pgid = None
        if pgid is not None:
            print(f"[stop_teleop] killpg(SIGTERM) pgid={pgid}")
            try:
                os.killpg(pgid, signal.SIGTERM)
                time.sleep(1.0)
            except ProcessLookupError:
                print("[stop_teleop] Group not found, likely ended.")
            else:
                try:
                    os.killpg(pgid, 0)
                    print(f"[stop_teleop] Still alive => killpg(SIGKILL) pgid={pgid}")
                    os.killpg(pgid, signal.SIGKILL)
                except ProcessLookupError:
                    print("[stop_teleop] Group ended after SIGTERM.")
        teleop_pid = None
    if teleop_pattern:
        print(f"[stop_teleop] pkill -9 -f {teleop_pattern}")
        try:
            subprocess.run(["pkill", "-9", "-f", teleop_pattern], check=False)
        except Exception as e:
            print(f"[stop_teleop] pkill error: {e}")
    teleop_pattern = None

def sync_home(robot):
    robot.SwitchMode(flexivrdk.Mode.NRT_PRIMITIVE_EXECUTION)
    robot.ExecutePrimitive("MoveJ", {"target": flexivrdk.JPos(HOME_POSE, [0]*6)})
    print(f"[SyncPose] 同步到 Home Pose: {HOME_POSE}")
    time.sleep(2)

def measure_stiffness_for_axis(leader_robot, slave_robot, axis):
    """
    持续采样，不设固定持续时长，直到连续 stable_count 段（每段 segment_duration 秒）
    的局部刚度值均大于 min_stiffness 且波动范围小于 max_fluctuation。
    局部刚度：对当前采样段数据计算：
      - 对平移方向，误差为主从 TCP 在该轴的差值 (m)；
      - 对旋转方向，先将四元数转换为 Euler 角，然后取对应角度差 (rad)。
      同时计算从手与主手在该轴的外力差值 F_diff。
      局部刚度 K_seg = (平均 |F_diff|) / (平均 |Δ|)。
    当连续 stable_count 个段满足条件时，返回这 stable_count 段的平均刚度。
    """
    stable_window = []
    segment_logs = []
    while True:
        seg_samples = []
        seg_start = time.time()
        while time.time() - seg_start < segment_duration:
            master_pose = leader_robot.states().tcp_pose.copy()  # [x,y,z,qw,qx,qy,qz]
            slave_pose = slave_robot.states().tcp_pose.copy()
            if TEST_AXES[axis]["type"] == "linear":
                delta = slave_pose[TEST_AXES[axis]["index"]] - master_pose[TEST_AXES[axis]["index"]]
            else:
                master_euler = quat_to_euler(master_pose[3:7])
                slave_euler  = quat_to_euler(slave_pose[3:7])
                delta = slave_euler[TEST_AXES[axis]["index"]] - master_euler[TEST_AXES[axis]["index"]]
            F_master = leader_robot.states().ext_wrench_in_world[TEST_AXES[axis]["index"]]
            F_slave  = slave_robot.states().ext_wrench_in_world[TEST_AXES[axis]["index"]]
            # F_diff = abs(F_slave - F_master)
            # seg_samples.append((abs(delta), F_diff))
            seg_samples.append((delta, F_slave))
            if abs(delta) < 1e-6:
                K_temp = float('inf')
            else:
                K_temp = F_slave/delta
            print(f"\r当前位置差:{delta: .2f} 当前F_slave {F_slave: .2f} 当前刚度{K_temp: .2f}", end="", flush=True)
            time.sleep(sample_interval)
        if seg_samples:
            N = len(seg_samples)
            avg_delta = sum(d for (d, F) in seg_samples) / N
            avg_Fdiff = sum(F for (d, F) in seg_samples) / N
            if abs(avg_delta) < 1e-6:
                K_seg = float('inf')
            else:
                K_seg = avg_Fdiff / avg_delta
            segment_logs.append(K_seg)
            stable_window.append(K_seg)
            # 保持最近 stable_count 个段
            if len(stable_window) > stable_count:
                stable_window.pop(0)
            if len(stable_window) == stable_count:
                # 判断是否满足条件：所有段均大于 min_stiffness 且波动范围小于 max_fluctuation
                if all(val > min_stiffness for val in stable_window):
                    fluct = max(stable_window) - min(stable_window)
                    if fluct < max_fluctuation:
                        print(f"\n稳定条件满足：连续 {stable_count} 段刚度 = {[f'{v:.1f}' for v in stable_window]}")
                        return sum(stable_window)/stable_count, avg_delta, segment_logs

def signal_handler(sig, frame):
    print("\n检测到中断，程序退出。")
    stop_teleop()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def main():
    global teleop_pid, leader_sn, follower_sn, SUDO_PASSWORD, nTrials
    args = parse_args()
    leader_sn = args.leader
    follower_sn = args.follower
    SUDO_PASSWORD = args.password
    trials_per_axis = args.num

    exes = find_executables()
    if not exes:
        print("当前目录无 test_ 开头的可执行程序。")
        sys.exit(1)
    print("可用遥操作程序:")
    for i, (fn, _) in enumerate(exes):
        print(f"  {i}: {fn}")
    idx = int(input("请选择要测试的程序序号: "))
    exe_path = exes[idx][1]

    leader_robot = flexivrdk.Robot(leader_sn)
    slave_robot = flexivrdk.Robot(follower_sn)

    print("同步到 Home Pose...")
    sync_home(leader_robot)
    sync_home(slave_robot)
    input("Home Pose 已同步，请按 Enter 启动遥操作程序...")

    start_teleop(exe_path)
    print("遥操作程序已启动，等待7秒稳定...")
    time.sleep(7)

    results = {}
    logs = {}
    for axis in TEST_AXES.keys():
        print(f"\n========== 测试 {axis} 方向的跟踪刚度 ==========")
        trial_values = []
        trial_logs = []
        for i in range(trials_per_axis):
            print(f"开始第 {i+1} 次测试，在{axis}方向作相对位移并保持相对静止，...")
            print(f"采样 {axis} 方向数据，请保持施力……")
            K, avg_delta, seg_log = measure_stiffness_for_axis(leader_robot, slave_robot, axis)
            unit = "N/m" if TEST_AXES[axis]["type"]=="linear" else "Nm/rad"
            print(f"第 {i+1} 次 {axis} 方向刚度 = {K:.1f} {unit}（平均误差 = {avg_delta:.4f}）")
            trial_values.append(K)
            trial_logs.append(seg_log)
            time.sleep(1)
        avg_K = sum(trial_values) / len(trial_values)
        results[axis] = (trial_values, avg_K)
        logs[axis] = trial_logs
        input('该方向采集完成，请复位后按enter键开启下一次测试')
    
    print("\n========== 各方向测试结果 ==========")
    for axis, (vals, avg_K) in results.items():
        unit = "N/m" if TEST_AXES[axis]["type"]=="linear" else "Nm/rad"
        print(f"{axis}: 试验值 = {[f'{v:.1f}' for v in vals]}, 平均刚度 = {avg_K:.1f} {unit}")
    
    now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = f"tracking_stiffness_summary_{now_str}.csv"
    with open(csv_filename, "w", newline='', encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["Tracking Stiffness Measurement Summary", now_str])
        writer.writerow(["Axis", "Trial Stiffness Values", "Average Stiffness"])
        for axis, (vals, avg_K) in results.items():
            unit = "N/m" if TEST_AXES[axis]["type"]=="linear" else "Nm/rad"
            seg_logs_str = "; ".join([", ".join(f"{v:.1f}" for v in log) for log in logs[axis]])
            writer.writerow([axis, ", ".join(f"{v:.1f}" for v in vals), f"{avg_K:.1f} {unit}", seg_logs_str])
    print(f"测试结果已保存到 {csv_filename}。")
    stop_teleop()
    print(f'遥操作程序已停止')

if __name__=="__main__":
    main()
