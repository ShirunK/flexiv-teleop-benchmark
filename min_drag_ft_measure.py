#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
drag_measure.py

功能：
1. 通过命令行参数 (-1, -2, -p, -n) 获取主/从机械臂序列号、sudo 密码（用于启动和停止 Teleop 程序）以及每个方向的测试次数（默认5次）。
   需要 sudo 密码的原因：启动和停止 Teleop 程序时需要以 root 权限调用 kill 命令，确保能够结束 Teleop 及其所有子进程。
2. 程序从当前目录中查找以 "test_" 开头的可执行文件，由用户选择后启动遥操作程序（只启动一次）。
3. 同步机器人到 Home Pose（预设 Home Pose），并等待用户确认后开始测试。
4. 对主机械臂末端在六个自由度（X, Y, Z, Rx, Ry, Rz）分别进行测试：
   - 每个方向测试时，提示用户踩下踏板后缓慢拖动主机械臂末端，
     当对应速度分量超过阈值（平移：0.01 m/s；旋转：0.01 rad/s）时，
     记录当时主侧的外力（或转矩）的绝对值作为本次测试的“最小拖拽力/转动扭矩”。
   - 每个方向重复测试 n 次，每次测试后提示用户将机械臂复位到 Home Pose并按 Enter。
5. 输出各方向的单次测试结果和平均值，并将所有结果保存到 CSV 文件中。
6. 程序中加入异常处理和 Ctrl+C 捕捉，一旦出错或中断，将停止 Teleop 程序（包括所有子进程）后退出。

"""

import argparse
import time
import signal
import sys
import csv
import os
import subprocess
from datetime import datetime

import flexivrdk

teleop_pid = None
teleop_pattern = None

# 测试参数
sample_interval = 0.01  # 采样周期 (100Hz)
linear_threshold = 0.01   # 平移速度阈值 (m/s)
angular_threshold = 0.1  # 旋转速度阈值 (rad/s) 末端手柄长度为15cm，假设杠杆臂为10cm，对应0.01m/s的角速度是0.1 rad/s
nTrials = 5  # 每个方向测试次数

# Home Pose（单位：度）
HOME_POSE = [-5.6850536735238373e-05, -39.999988598597405, -7.796941005345694e-05,
             89.99967467229428, -1.394160247868911e-05, 39.99993054198945, -1.9290991341998603e-06]

# 测试自由度配置：平移使用 tcp_vel 索引 0,1,2；旋转使用索引 3,4,5
# 旋转测试使用相同的 Home Pose
TEST_AXES = {
    "X": {"index": 0, "is_rotation": False},
    "Y": {"index": 1, "is_rotation": False},
    "Z": {"index": 2, "is_rotation": False},
    "Rx": {"index": 3, "is_rotation": True},
    "Ry": {"index": 4, "is_rotation": True},
    "Rz": {"index": 5, "is_rotation": True},
}

def parse_args():
    parser = argparse.ArgumentParser(description="Drag Force/Torque Measurement for Master")
    parser.add_argument("-1", "--leader", required=True, help="主机械臂序列号")
    parser.add_argument("-2", "--follower", required=True, help="从机械臂序列号")
    parser.add_argument("-p", "--password", required=True, help="用于启动和停止 Teleop 程序的 sudo 密码")
    parser.add_argument("-n", "--num", type=int, default=5, help="每个方向测试次数 (默认5次)")
    return parser.parse_args()

def find_executables_in_current_dir():
    files = os.listdir('.')
    candidates = []
    for f in files:
        if os.path.isfile(f) and os.access(f, os.X_OK) and f.startswith("test_"):
            candidates.append((f, f"./{f}"))
    return candidates

def start_teleop(executable_path):
    """
    启动 Teleop 程序，并使其成为新会话的组长。
    若文件名包含 "high_transparency"，则使用 -l / -r 参数，否则使用 -1 / -2 参数。
    返回 Teleop 进程的 PID。
    """
    global teleop_pid, teleop_pattern, SUDO_PASSWORD, leader_robot_sn, follower_robot_sn
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

def sync_home_pose(robot):
    robot.SwitchMode(flexivrdk.Mode.NRT_PRIMITIVE_EXECUTION)
    home = flexivrdk.JPos(HOME_POSE, [0,0,0,0,0,0])
    robot.ExecutePrimitive("MoveJ", {"target": home})
    print(f"[SyncPose] 同步到 Home Pose: {HOME_POSE}")
    time.sleep(2.0)

def measure_drag_for_axis(axis_name, idx, is_rotation):
    """
    对指定轴（例如 "X", "Y", "Z", "Rx", "Ry", "Rz"）进行测试：
    - 等待用户踩下踏板（digital_inputs()[0][0]==1）；
    - 当对应速度分量超过阈值（平移：tcp_vel[idx]；旋转：tcp_vel[idx]）时，
      记录主侧外力（或转矩）的绝对值作为“最小拖拽力/转动扭矩”。
    每个方向测试 nTrials 次，每次测试后提示用户将机械臂复位到 Home Pose并按 Enter 确认。
    返回：试验结果列表和平均值。
    """
    trials = []
    threshold = angular_threshold if is_rotation else linear_threshold
    for t in range(nTrials):
        print(f"\n[{axis_name}方向] 第 {t+1} 次测试：")
        print("请踩下踏板后，缓慢拖动主机械臂末端沿该方向运动，直至检测到运动启动。")
        # 等待用户踩下踏板
        while True:
            try:
                pedal = leader_robot.digital_inputs()[0]
            except Exception:
                pedal = 0
            if pedal == 1:
                break
            time.sleep(0.05)
        print("踏板已踩下，等待运动启动...")
        moving = False
        measured_value = None
        while not moving:
            states = leader_robot.states()
            velocity = states.tcp_vel
            if abs(velocity[idx]) >= threshold:
                moving = True
                wrench = states.ext_wrench_in_world
                measured_value = abs(wrench[idx])
                unit = "Nm" if is_rotation else "N"
                print(f"检测到运动，记录值 = {measured_value:.4f} {unit}")
            time.sleep(0.01)
        input("采集完成，此时您可以遥操机械臂到合适的POSE，如HOME POSE后按 Enter，继续下一次测试...")
        trials.append(measured_value)
    avg_val = sum(trials) / len(trials) if trials else 0.0
    unit = "Nm" if is_rotation else "N"
    print(f"\n[{axis_name}方向] 试验值: {['{:.4f}'.format(x) for x in trials]}, 平均 = {avg_val:.4f} {unit}")
    return trials, avg_val

def safe_exit():
    stop_teleop()
    sys.exit(0)

def signal_handler(sig, frame):
    print("\n[Ctrl+C] 检测到中断，停止 Teleop 并退出程序。")
    safe_exit()

import argparse
signal.signal(signal.SIGINT, signal_handler)

def main():
    global teleop_pid, leader_robot_sn, follower_robot_sn, SUDO_PASSWORD, nTrials, leader_robot, follower_robot

    args = parse_args()
    leader_robot_sn = args.leader
    follower_robot_sn = args.follower
    SUDO_PASSWORD = args.password
    nTrials = args.num

    exe_list = find_executables_in_current_dir()
    if not exe_list:
        print("当前目录无 test_ 开头的可执行程序。")
        sys.exit(1)
    print("可用遥操作程序:")
    for i, (fn, _) in enumerate(exe_list):
        print(f"  {i}: {fn}")
    choice = input("请选择要测试的程序序号: ")
    try:
        cidx = int(choice)
        if cidx < 0 or cidx >= len(exe_list):
            print("无效选择.")
            sys.exit(1)
    except:
        print("输入错误.")
        sys.exit(1)
    chosen_name, exe_path = exe_list[cidx]
    print(f"\n已选择: {chosen_name}\n路径: {exe_path}\n")
    
    print("连接到 Robot...")
    leader_robot = flexivrdk.Robot(leader_robot_sn)
    follower_robot = flexivrdk.Robot(follower_robot_sn)
    
    print("同步到 Home Pose...")
    sync_home_pose(leader_robot)
    sync_home_pose(follower_robot)
    input("Home Pose 已同步，按 Enter 开始启动 Teleop 程序...")
    
    start_teleop(exe_path)
    print("遥操作程序已启动，等待7秒稳定...")
    time.sleep(7.0)
    
    # 对6个自由度进行测试：平移使用索引 0,1,2；旋转使用索引 3,4,5
    results = {}
    for axis, cfg in TEST_AXES.items():
        print(f"\n========== 测试 {axis} 方向的最小 {'转矩' if cfg['is_rotation'] else '拖拽力'} ==========")
        print(f"请按提示操作：踩下踏板后，缓慢拖动主机械臂末端沿 {axis} 方向运动，直到检测到运动。")
        trials, avg_val = measure_drag_for_axis(axis, cfg["index"], cfg["is_rotation"])
        results[axis] = (trials, avg_val)
    
    # 输出所有结果
    print("\n========== 各方向测试结果 ==========")
    for axis, (trials, avg_val) in results.items():
        unit = "Nm" if axis.startswith("R") else "N"
        print(f"{axis}: 试验值 = {['{:.4f}'.format(x) for x in trials]}, 平均 = {avg_val:.4f} {unit}")
    
    # 保存结果到 CSV 文件
    now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = f"drag_measure_summary_{now_str}.csv"
    with open(csv_filename, "w", newline='', encoding="utf-8") as fcsv:
        writer = csv.writer(fcsv)
        writer.writerow(["Drag/Torque Measurement Summary", now_str])
        writer.writerow(["Axis", "Trial Values", "Average Value"])
        for axis, (trials, avg_val) in results.items():
            unit = "Nm" if axis.startswith("R") else "N"
            writer.writerow([axis, ", ".join("{:.4f}".format(x) for x in trials), f"{avg_val:.4f} {unit}"])
    print(f"测试结果已保存到 {csv_filename}。")
    
    # 停止 Teleop 程序
    stop_teleop()
    print("遥操作程序已停止，程序结束。")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nCtrl+C detected. Exiting drag_measure.py.")
        sys.exit(0)
