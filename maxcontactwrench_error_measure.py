#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
maxcontactwrench_error_measure.py

功能：
1. 通过命令行参数 (-1, -2, -n) 获取主/从机械臂序列号。
2. 需要用户提前准备好可以切换max_contact_wrench的遥操作程序例程。
3. 同步主/从机械臂到 Home Pose，然后等待用户启动例程，并按 Enter 开始测试。
4. 提示用户输入最大接触力限制设定值（单位 N）。
5. 在遥操作运行过程中，提示用户用主手向下施加远大于设定值的力，使从手末端接触外界；
   当检测到主手 Z 方向外力连续 3 秒大于 20 N 时，在该区间内采集从手 Z 方向外力数据，并计算平均值。
6. 每次测试结束后，重复测试 3 次；最后输出每次测试结果及平均值，并保存到 CSV 文件中。
"""

import argparse
import time
import signal
import sys
import csv
from datetime import datetime

import flexivrdk

# 测试参数
valid_duration = 3.0            # 连续有效时长 (秒)
nTests = 3                      # 测试次数，默认为3

def parse_args():
    parser = argparse.ArgumentParser(description="Max Contact Wrench Error Measurement")
    parser.add_argument("-1", "--leader", required=True, help="主机械臂序列号")
    parser.add_argument("-2", "--follower", required=True, help="从机械臂序列号")
    return parser.parse_args()

def measure_max_contact_error(leader_robot, follwer_robot, set_value):
    """
    测试方法：
      提示用户用主手向下施加大于设定值的力，使从手末端接触外界。
      当检测到主侧 Z 方向外力连续 3 秒大于 set_value 时，
      在该区间内采集从手 Z 方向外力数据，并计算平均值。
    返回：平均从侧力及误差百分比
    """
    print(f"请用主手向下施加大于 {set_value:.1f} N 的力，使从手末端接触外界。")
    valid_start = None
    slave_values = []
    while True:
        F_master = leader_robot.states().ext_wrench_in_world[2]
        F_slave = follwer_robot.states().ext_wrench_in_world[2]
        print(f"\r主侧力 = {F_master: .2f} N, 从侧力 = {F_slave: .2f} N", end="", flush=True)
        if abs(F_master) >= set_value:
            if valid_start is None:
                valid_start = time.time()
                slave_values = []
            slave_values.append(F_slave)
        else:
            valid_start = None
            slave_values = []
        if valid_start is not None and (time.time() - valid_start) >= valid_duration:
            print("检测到主侧力大于threshold持续3秒。")
            break
        time.sleep(0.01)
    avg_slave = sum(slave_values)/len(slave_values) if slave_values else 0.0
    error_percent = abs(avg_slave - set_value)/set_value * 100 if set_value != 0 else float('inf')
    print(f"测得平均从侧力: {avg_slave:.4f} N, 设定值: {set_value:.4f} N, 误差: {error_percent:.2f}%")
    return avg_slave, error_percent

def signal_handler(sig, frame):
    print("\n检测到中断。程序退出。")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def main():
    args = parse_args()
    leader_robot = flexivrdk.Robot(args.leader)
    follower_robot = flexivrdk.Robot(args.follower)

    input("请手动启动test_high_transparency_teleop_switch_contact_wrench，启动完成后按 Enter 继续...")
    print("test_high_transparency_teleop_switch_contact_wrench例程r键engage, x,y,z键可以调整 maxcontactwrench to 15.0, 5.0, 1.0")
    set_value = float(input("请输入当前设置的最大接触力限制设定值 (单位 N): "))
    test_results = []
    for i in range(nTests):
        avg_slave, error = measure_max_contact_error(leader_robot, follower_robot, set_value)
        test_results.append((avg_slave, error))
        print(f"第 {i+1} 次测试：平均从侧力 = {avg_slave:.4f} N，误差 = {error:.2f}%")
        input("请抬起机械臂后按 Enter 继续下一次测试...")

    avg_all = sum(x[0] for x in test_results) / len(test_results)
    err_all = sum(x[1] for x in test_results) / len(test_results)
    print("\n========== 测试结果 ==========")
    for i, (val, err) in enumerate(test_results, 1):
        print(f"第 {i} 次：平均从侧力 = {val:.4f} N，误差 = {err:.2f}%")
    print(f"总体平均从侧力 = {avg_all:.4f} N，平均误差 = {err_all:.2f}%")
    
    now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = f"maxcontactwrench_summary_{now_str}.csv"
    with open(csv_filename, "w", newline='', encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["Max Contact Wrench Error Measurement Summary", now_str])
        writer.writerow(["Test Number", "Average Slave Force (N)", "Error (%)"])
        for i, (val, err) in enumerate(test_results, 1):
            writer.writerow([i, f"{val:.4f}", f"{err:.2f}"])
        writer.writerow([])
        writer.writerow(["Overall Average Slave Force (N)", f"{avg_all:.4f}"])
        writer.writerow(["Overall Error (%)", f"{err_all:.2f}"])
    print(f"测试结果已保存到 {csv_filename}。")

if __name__=="__main__":
    main()
