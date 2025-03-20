#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
transparency_measure.py

功能：
1. 程序启动时提示“现在请运行要测试的遥操作程序，运行好后按Enter”，并列出当前目录中以“test_”开头的可执行程序，
   由用户选择后，由程序启动遥操作程序（启动后不在各次测试间重启）。
2. 测试过程中提示“请操控主手，使末端触碰到平面，并尝试使末端保持10N的压力并维持3秒”，
   系统以一定频率（例如10Hz）读取主侧与从侧末端在 world 坐标下的 Z 方向外力，
   实时计算透明度指标：F = - F_master_z / F_slave_z，其中 F 为透明度，F_master_z 为主侧 Z 方向外力，
   并以“slave:master”显示,当从侧力连续3秒保持在9N到11N之间时，   记录该区间数据并计算平均值，作为单次测试的结果。
3. 整个测试过程连续进行 n 次（默认 n=5），每次测试结束后输出该次结果，
   最后输出 n 次测试结果的平均值，并将所有数据和总结保存到 CSV 文件中。
"""

import time
import signal
import sys
import csv
import os
import subprocess
from datetime import datetime
import argparse
import flexivrdk

# 全局变量：Teleop进程PID及其匹配模式
teleop_pid = None
teleop_pattern = None

# 机器人连接参数
leader_robot_sn = None
follower_robot_sn = None
SUDO_PASSWORD = None

# 测试参数
finalDistM = 0.30  # 目标位移 (米)，例如0.30表示30cm
startDist  = 0.05  # 从5cm开始采样
sample_interval = 0.1  # 采样周期，0.1秒，即10Hz
valid_duration = 3.0  # 连续有效时间3秒 

# =========== 函数：查找可执行文件 ===========

def find_executables_in_current_dir():
    files = os.listdir('.')
    candidates = []
    for f in files:
        if os.path.isfile(f) and os.access(f, os.X_OK):
            if f.startswith("test_"):
                candidates.append((f, f"./{f}"))
    return candidates

# =========== 函数：启动 & 停止 Teleop ===========

def start_teleop(executable_path):
    """
    启动Teleop进程，并使其成为新会话的组长。
    若文件名包含"high_transparency"，则使用参数 -l / -r，
    否则使用 -1 / -2。返回启动进程的PID。
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
    尝试通过 killpg(SIGTERM+SIGKILL) 结束Teleop进程组，
    然后调用 pkill -9 -f 以确保所有相关进程被关闭。
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

def measure_transparency_once():
    """
    在循环中以固定频率读取主臂与从臂末端 Z 方向外力，
    实时计算透明度 T = -F_master_z / F_slave_z（格式：slave:master），
    并提示：若 F_slave_z < 9N 输出“请大力一些”，大于 11N 输出“请小力一些”。
    当从侧力连续有效3秒（9N<=F_slave_z<=11N）后，记录该区间数据，
    并计算平均透明度。
    返回：平均透明度 T_avg。
    """
    valid_start_time = None
    valid_data = []  # 存储有效采样数据： (timestamp, F_master_z, F_slave_z)
    
    while True:
        leader_states = leader_robot.states()
        follower_states = follower_robot.states()
        
        F_master_z = leader_states.ext_wrench_in_world[2]
        F_slave_z = follower_states.ext_wrench_in_world[2]
        
        if abs(F_slave_z) < 1e-6:
            T = float('inf')
        else:
            T = -F_master_z / F_slave_z
        
        msg = f"F_slave_z = {F_slave_z: .4f} N, F_master_z = {F_master_z: .4f} N, 透明度 = 1:{T:.4f}"
        if F_slave_z < 9.0:
            msg += "    --> 请大力一些"
        elif F_slave_z > 11.0:
            msg += "    --> 请小力一些"
        else:
            msg += "    --> 请保持3秒"
        print("\r" + msg.ljust(80), end="", flush=True)
        
        current_time = time.time()
        if 9.0 <= F_slave_z <= 11.0:
            if valid_start_time is None:
                valid_start_time = current_time
                valid_data = []
            valid_data.append((current_time, F_master_z, F_slave_z))
        else:
            valid_start_time = None
            valid_data = []
        
        if valid_start_time is not None and (current_time - valid_start_time) >= valid_duration:
            print("\n连续有效3秒，采集结束。")
            break
        
        time.sleep(sample_interval)
    
    if not valid_data:
        print("未采集到有效数据，返回无效结果。")
        return None
    
    N = len(valid_data)
    sum_F_master = sum(fm for (_, fm, _) in valid_data)
    sum_F_slave  = sum(fs for (_, _, fs) in valid_data)
    avg_F_master = sum_F_master / N
    avg_F_slave  = sum_F_slave / N
    
    if abs(avg_F_slave) < 1e-6:
        T_avg = float('inf')
    else:
        T_avg = -avg_F_master / avg_F_slave
    
    print(f"\n最终有效区间平均：F_slave_z = {avg_F_slave: .4f} N, F_master_z = {avg_F_master: .4f} N")
    print(f"透明度 (slave:master) = 1:{T_avg:.4f}")
    return T_avg

# =========== 异常 / Ctrl+C 处理 ===========
def safe_exit():
    stop_teleop()
    sys.exit(0)

def signal_handler(sig, frame):
    print("\n[Ctrl+C] Caught, stopping teleop and exiting.")
    safe_exit()

signal.signal(signal.SIGINT, signal_handler)

def main():
    global teleop_pid, nTests, leader_robot_sn, follower_robot_sn, SUDO_PASSWORD

    parser = argparse.ArgumentParser(description="Transparency Measurement for Master Force Feedback")
    parser.add_argument("-1", "--leader", required=True, help="主机械臂序列号")
    parser.add_argument("-2", "--follower", required=True, help="从机械臂序列号")
    parser.add_argument("-p", "--password", required=True, help="用于启动和停止 Teleop 程序的 sudo 密码")
    parser.add_argument("-n", "--num", type=int, default=5, help="连续测试次数 (默认5次)")
    args = parser.parse_args()

    leader_robot_sn = args.leader
    follower_robot_sn = args.follower
    SUDO_PASSWORD = args.password
    nTests = args.num

    # 1) 搜索可执行文件
    exe_list = find_executables_in_current_dir()
    if not exe_list:
        print("当前目录无 test_ 开头的可执行程序。")
        sys.exit(1)
    
    print("可用遥操作程序:")
    for i, (fn, fp) in enumerate(exe_list):
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
    
    # 2) 连接Robot
    print("连接到 Robot...")
    # 这里创建两个实例
    global leader_robot, follower_robot
    leader_robot = flexivrdk.Robot(leader_robot_sn)
    follower_robot = flexivrdk.Robot(follower_robot_sn)
    
    # 3) 启动遥操作程序（只启动一次）
    start_teleop(exe_path)
    
    # 4) 询问连续测试次数 (默认5次)
    time.sleep(7.0)
    print(f"将连续测试 {nTests} 次...")
    
    test_results = []
    for i in range(nTests):
        print(f"\n---------- 第 {i+1} 次测试 ----------")
        print("请操控主手，使末端触碰到平面，并尝试使末端保持约10N压力并维持3秒。")
        T_avg = measure_transparency_once()
        if T_avg is not None:
            test_results.append(T_avg)
            print(f"第 {i+1} 次测试透明度 = 1:{T_avg:.4f}")
        else:
            print(f"第 {i+1} 次测试无效。")
        time.sleep(1.0)
    
    # 计算n次测试平均结果
    if test_results:
        avg_result = sum(test_results) / len(test_results)
        print("\n========== 测试结果 ==========")
        for idx, res in enumerate(test_results):
            print(f"第 {idx+1} 次透明度 = 1:{res:.4f}")
        print(f"平均透明度 = 1:{avg_result:.4f}")
    else:
        print("没有有效的测试数据。")
    
    # 保存结果到 CSV
    now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = f"transparency_summary_{now_str}.csv"
    with open(csv_filename, "w", newline='', encoding="utf-8") as fcsv:
        writer = csv.writer(fcsv)
        writer.writerow(["Transparency Measurement Summary", now_str])
        writer.writerow(["Test Number", "Transparency (1:T)"])
        for idx, res in enumerate(test_results):
            writer.writerow([idx+1, f"1:{res:.4f}"])
        writer.writerow([])
        if test_results:
            writer.writerow(["Average Transparency (1:T)", f"1:{avg_result:.4f}"])
    print(f"测试结果已保存到 {csv_filename}。")
    
    # 5) 停止遥操作程序
    print(f"即将停止遥操作程序，建议使其远离接触物体。")
    time.sleep(3)
    stop_teleop()
    print("程序结束。")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nCtrl+C detected. Exiting transparency_measure.py.")
        sys.exit(0)
