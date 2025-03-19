#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
float_offset_measure.py

功能：
1. 通过命令行参数 (-1, -2, -p, -n) 获取主/从机械臂序列号、sudo 密码（用于启动和停止 Teleop 程序），以及测试次数（默认10次）。
   需要 sudo 密码的原因：启动和停止 Teleop 程序时必须以 root 权限执行 kill 命令，以确保彻底终止 Teleop 及其所有子进程。
2. 从当前目录列出所有以 "test_" 开头的可执行文件，由用户选择后启动遥操作程序（只启动一次）。
3. 同步主从机械臂到 Home Pose，然后等待用户按 Enter 开始测试。
4. 自动（tdk1.2.2）或手动（TransparencyCart）移动机械臂到测试Pose，踩下踏板后记录 1 秒前后 TCP 位移（mm），并判断 <10mm 为成功。
5. 输出每次位移、成功/失败；最后计算成功率和平均位移，保存所有结果到 CSV。
6. 程序结束时停止遥操作程序，并优雅退出。

"""

import argparse, time, signal, sys, csv, os, subprocess
from datetime import datetime
import flexivrdk
import math

teleop_pid = None
teleop_pattern = None

HOME_POSE = [-5.6850536735238373e-05, -39.999988598597405, -7.796941005345694e-05,
             89.99967467229428, -1.394160247868911e-05, 39.99993054198945, -1.9290991341998603e-06]

test_pose = [
    [-5.6850536735238373e-05, -39.999988598597405, -7.796941005345694e-05, 89.99967467229428, -1.394160247868911e-05, 39.99993054198945, -1.9290991341998603e-06],
    [-1.9044804296524143, -6.446138733250514, 1.7813227273838543, 105.78366811126669, 0.01993463080299201, 22.22781246922564, -0.08457543773347569],
    [29.045693639633996, -35.54101229623877, 1.3417330329797759, 114.2940633281328, -63.59731205327565, -15.533069364291974, 64.116044430268],
    [-3.2642652257251, -49.085789687104665, 2.0678128777397085, 112.8067213341916, -6.5256809819053, 156.3619360945694, -4.714375858493576],
    [1.7785820072571665, -41.79144476539052, -0.44597925633254565, 80.59858180635628, -1.4068501355481982, -55.52552426423344, -20.594531530808666],
    [-29.870056491474276, -38.30158692907401, -24.50869096372978, 81.13731297724217, 66.41504512419088, -47.28616461406268, -67.86525137960776],
    [-19.548061172401624, -41.799138973491985, -16.676540361762452, 114.56878719698062, -156.30846937373647, -62.92388272174265, 12.092397427745253],
    [-52.16029688972555, 5.5811050931029325, -31.065633294585272, 127.25281731829074, 39.33474866388078, 35.633667227441975, -99.20402393915312],
    [53.2187576452228, -86.36379422168464, -81.1679532058646, 112.68209770257214, -18.790241438562685, 67.06243094441373, 72.64664334670279],
    [-72.04356496363411, -89.724056048665, 86.44702007673688, 66.80374435974011, -2.3625400227927327, -6.845101608690083, -59.72234766091402],
]

def parse_args():
    p = argparse.ArgumentParser(description="Float Offset (Hover) Success Rate Measurement")
    p.add_argument("-1","--leader", required=True, help="主机械臂序列号")
    p.add_argument("-2","--follower", required=True, help="从机械臂序列号")
    p.add_argument("-p","--password", required=True, help="sudo 密码，用以开启关闭遥操作")
    p.add_argument("-n","--num", type=int, default=10, help="测试次数 (默认10次)")
    return p.parse_args()

def find_executables():
    return [(f,f"./{f}") for f in os.listdir('.') if f.startswith("test_") and os.access(f,os.X_OK)]

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

def move_j_deg(robot, pose_deg):
    robot.SwitchMode(flexivrdk.Mode.NRT_PRIMITIVE_EXECUTION)
    robot.ExecutePrimitive("MoveJ", {"target": flexivrdk.JPos(pose_deg,[0]*6),"jntVelScale": 25})

def is_reached_joint_pose(robot, pose_deg, joint_allowing_error_deg):
    target = [math.radians(x) for x in pose_deg]
    tol = math.radians(joint_allowing_error_deg)
    current = robot.states().q  # list of 7 floats in radians
    return all(abs(curr - tgt) <= tol for curr, tgt in zip(current, target))

def wait_for_reached_or_timeout(robot, pose_deg, joint_allowing_error_deg, time_out):
    start_time = time.time()
    while True:
        if is_reached_joint_pose(robot, pose_deg, joint_allowing_error_deg) or time.time() - start_time >= time_out:
            break
        time.sleep(0.2)

def measure_hover(robot):
    start = robot.states().tcp_pose.copy()
    time.sleep(1)
    end = robot.states().tcp_pose.copy()
    dx = (end[0]-start[0])*1000
    dy = (end[1]-start[1])*1000
    dz = (end[2]-start[2])*1000
    dist = (dx*dx + dy*dy + dz*dz)**0.5
    return round(dist,2), dist<10.0

def safe_exit():
    stop_teleop()
    sys.exit(0)

signal.signal(signal.SIGINT, lambda s,f: safe_exit())

def main():
    global teleop_pid, leader_robot_sn, follower_robot_sn, SUDO_PASSWORD
    args = parse_args()
    leader_robot_sn = args.leader
    follower_robot_sn = args.follower
    SUDO_PASSWORD = args.password
    tests = find_executables()
    if not tests:
        print("No test_ executables found."); sys.exit(1)
    for i,(fn,_) in enumerate(tests): print(f"{i}: {fn}")
    idx=int(input("Select program index: "))
    exe_path = tests[idx][1]
    teleop_pattern = os.path.basename(exe_path)
    if "high_transparency" in teleop_pattern:
        is_auto = False
        print('high_transparency当前不支持同时move_j，需要手动移动到测试Pose')
    else:
        is_auto = True
        print('当前测试teleop支持自动移动到测试pose.')
    leader = flexivrdk.Robot(args.leader)
    follower = flexivrdk.Robot(args.follower)
    print("Sync Home Pose...")
    current_mode = leader.mode()
    move_j_deg(leader, HOME_POSE)
    move_j_deg(follower, HOME_POSE)
    wait_for_reached_or_timeout(leader, HOME_POSE, 2, 7)
    wait_for_reached_or_timeout(follower, HOME_POSE, 2, 7)
    leader.SwitchMode(current_mode)
    follower.SwitchMode(current_mode)
    input("Home Pose synced. Press Enter to start Teleop...")
    start_teleop(exe_path)
    time.sleep(7)

    results=[]
    for i in range(args.num):
        if is_auto:
            print(f"请踩住踏板等待机器人前往测试POSE，等待提示松开后再松开，提前松开踏板会报FAULT_OVERSPEED！")
            while True:
                try:
                    pedal = leader.digital_inputs()[0]
                except Exception:
                    pedal = 0
                if pedal == 1:
                    break
                time.sleep(0.05)
            print(f"正在前往测试POSE...")
            teleop_mode = leader.mode()
            print(i)
            if i>0:
                move_j_deg(leader, test_pose[i-1])
                move_j_deg(follower, test_pose[i-1])
                wait_for_reached_or_timeout(leader, test_pose[i-1], 2, 5)
                wait_for_reached_or_timeout(follower, test_pose[i-1], 2, 5)
                move_j_deg(leader, test_pose[1])
                move_j_deg(follower, test_pose[1])
                wait_for_reached_or_timeout(leader, test_pose[1], 2, 5)
                wait_for_reached_or_timeout(follower, test_pose[1], 2, 5)
            move_j_deg(leader, test_pose[i])
            move_j_deg(follower, test_pose[i])
            wait_for_reached_or_timeout(leader, test_pose[i], 2, 5)
            wait_for_reached_or_timeout(follower, test_pose[i], 2, 5)
            print("到达测试Pose，请松开踏板")
            while True:
                try:
                    pedal = leader.digital_inputs()[0]
                except Exception:
                    pedal = 0
                if pedal == 0:
                    break
                time.sleep(0.05)
            leader.SwitchMode(teleop_mode)
            follower.SwitchMode(teleop_mode)
        else:
            input(f"请将末端移动到测试 Pose 后按 Enter 开始")
        print(f"第 {i+1} 次测试已开始，请踩住踏板并等待提示。")
        while True:
            try:
                pedal = leader.digital_inputs()[0]
            except Exception:
                pedal = 0
            if pedal == 1:
                break
            time.sleep(0.05)
        print("踏板已踩下，等待运动启动...")
        dist, success = measure_hover(leader)
        results.append((dist, success))
        print(f"Distance: {dist} mm — {'Success' if success else 'Fail'}")
        print(f"第 {i+1} 次测试已完成，请松开踏板。")
        while True:
            try:
                pedal = leader.digital_inputs()[0]
            except Exception:
                pedal = 1
            if pedal == 0:
                break
            time.sleep(0.05)
    stop_teleop()
    

    now=datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_name=f"hover_summary_{now}.csv"
    try:
        f = open(csv_name,"w",newline='', encoding="utf-8")
        w=csv.writer(f)
        w.writerow(["Test","Distance(mm)","Success"])
        for i,(d,s) in enumerate(results,1):
            w.writerow([i,d,"Yes" if s else "No"])
        avg_dist = round(sum(d for d,_ in results)/len(results),2)
        success_rate = round(sum(s for _,s in results)/len(results)*100,1)
        w.writerow([])
        w.writerow(["Average Distance(mm)",avg_dist])
        w.writerow(["Success Rate(%)",success_rate])
    except Exception as e:
        print(f"[Error] {e}")
    print(f"\nSaved results to {csv_name}")
    print(f"Average Distance: {avg_dist} mm, Success Rate: {success_rate}%")
    print("Done.")
    time.sleep(3)
    move_j_deg(leader, test_pose[9])
    move_j_deg(follower, test_pose[9])
    wait_for_reached_or_timeout(leader, test_pose[9], 2, 7)
    wait_for_reached_or_timeout(follower, test_pose[9], 2, 7)
    move_j_deg(leader, test_pose[1])
    move_j_deg(follower, test_pose[1])
    wait_for_reached_or_timeout(leader, test_pose[1], 2, 7)
    wait_for_reached_or_timeout(follower, test_pose[1], 2, 7)

if __name__=="__main__":
    main()
