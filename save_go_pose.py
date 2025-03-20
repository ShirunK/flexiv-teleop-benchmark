#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
save_go_pose_dual.py

功能：
1. 通过命令行位置参数获取一个或多个机器人序列号（例如：python save_go_pose_dual.py Rizon4s-123456 Rizon4s-123452）。
2. 脚本支持如下命令：
   - 按‘s’键保存第一个机器人当前的姿态（保存关节角、TCP姿态的四元数和Euler角）姿态信息写入 CSV 文件中。
   - 按‘g’键加载保存的姿态，依次使所有机器人移动到该姿态（调用 move_j_deg 和 wait_for_reached_or_timeout）。
   - 按‘h’键使所有机器人回 Home Pose。
   - 按'q'来退出

"""
import time
import flexivrdk
import csv
import os
import ast
import math
import argparse

PI = 3.141592653
HOME_POSE = [-5.6850536735238373e-05, -39.999988598597405, -7.796941005345694e-05,
             89.99967467229428, -1.394160247868911e-05, 39.99993054198945, -1.9290991341998603e-06]
filename = "save_pose.csv"

def move_j_deg(robot, pose_deg):
    robot.SwitchMode(flexivrdk.Mode.NRT_PRIMITIVE_EXECUTION)
    robot.ExecutePrimitive("MoveJ", {"target": flexivrdk.JPos(pose_deg,[0]*6),"jntVelScale": 15})

def is_reached_joint_pose(robot, pose_deg, joint_allowing_error_deg):
    target = [math.radians(x) for x in pose_deg]
    tol = math.radians(joint_allowing_error_deg)
    current = robot.states().q
    return all(abs(curr - tgt) <= tol for curr, tgt in zip(current, target))

def wait_for_reached_or_timeout(robot, pose_deg, joint_allowing_error_deg, time_out):
    start_time = time.time()
    while True:
        if is_reached_joint_pose(robot, pose_deg, joint_allowing_error_deg) or time.time() - start_time >= time_out:
            break
        time.sleep(0.2)

def quaternion_to_euler(qw, qx, qy, qz):
    t0 = +2.0 * (qw * qx + qy * qz)
    t1 = +1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (qw * qy - qz * qx)
    t2 = max(-1.0, min(1.0, t2))
    pitch = math.asin(t2)

    t3 = +2.0 * (qw * qz + qx * qy)
    t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(t3, t4)
    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]

def parse_args():
    parser = argparse.ArgumentParser(description="save_go_pose")
    parser.add_argument("robots", nargs="+", help="one or more Robot series number")
    return parser.parse_args()

if not os.path.exists(filename):
    with open(filename, mode="w", newline="") as file:
        csv_writer = csv.writer(file)
        csv_writer.writerow(["joint_positions_deg", "joint_positions_rad", "tcp_pose_quat", "tcp_pose_euler"])
    print("Created new file:", filename)

def main():
    args = parse_args()
    robot_sn_list = args.robots
    robots = [flexivrdk.Robot(sn) for sn in robot_sn_list]
    
    print("\n How are you! Use the following command to control robots sets")
    print("  s: Save the first robot's pose")
    print("  g: Move all robots to selected pose")
    print("  h: Home all robots")
    print("  q: Exit")
    
    while True:
        print("\nPress 's' to save pose, 'g' to go to a saved pose, 'h' to go home, or 'q' to quit.")
        key = input("Enter your choice: ").strip().lower()
        
        if key == 's':
            save_pose_rad = robots[0].states().q
            save_pose_deg = [angle * 180 / PI for angle in save_pose_rad]
            current_pose_tcp = robots[0].states().tcp_pose.copy()
            tcp_pose_quat = current_pose_tcp
            x, y, z = current_pose_tcp[0:3]
            qw, qx, qy, qz = current_pose_tcp[3:]
            euler_angles = quaternion_to_euler(qw, qx, qy, qz)
            tcp_pose_euler = [x, y, z] + euler_angles
            
            print(f"Current Robot:{robot_sn_list[0]}")
            print("Current Joint Position (rad):", save_pose_rad)
            print("Current Joint Position (deg):", save_pose_deg)
            print("Current TCP pose (quat):", tcp_pose_quat)
            print("Current TCP pose (euler deg):", tcp_pose_euler)
            
            with open(filename, mode="a", newline="") as file:
                csv_writer = csv.writer(file)
                csv_writer.writerow([str(save_pose_deg), str(save_pose_rad), str(tcp_pose_quat), str(tcp_pose_euler)])
            print("Pose has been saved to", filename)
            
        elif key == 'g':
            poses_deg = []
            poses_rad = []
            tcp_quats = []
            tcp_eulers = []
            with open(filename, mode="r", newline="") as file:
                csv_reader = csv.reader(file)
                header = next(csv_reader, None)
                for row in csv_reader:
                    try:
                        pose_deg = ast.literal_eval(row[0])
                        pose_rad = ast.literal_eval(row[1])
                        tcp_quat = ast.literal_eval(row[2])
                        tcp_euler = ast.literal_eval(row[3])
                    except Exception as e:
                        print("Error in decode pose：", row, e)
                        continue
                    poses_deg.append(pose_deg)
                    poses_rad.append(pose_rad)
                    tcp_quats.append(tcp_quat)
                    tcp_eulers.append(tcp_euler)
            if not poses_deg:
                print("No saved pose in csv file")
                continue
            print("Saved Pose:")
            for idx, pose in enumerate(poses_deg):
                print(f"{idx}: {pose} | TCP (quat): {tcp_quats[idx]} | TCP (euler): {tcp_eulers[idx]}")
            try:
                index = int(input("Please input the index of pose to move ").strip())
                if index < 0 or index >= len(poses_deg):
                    print("invalid input!")
                    continue
            except ValueError:
                print("invalid input!")
                continue
            
            target_pose_deg = poses_deg[index]
            current_modes = [robot.mode() for robot in robots]
            print("moving to pose (deg):", target_pose_deg)
            for robot in robots:
                move_j_deg(robot, target_pose_deg)
            for robot in robots:
                wait_for_reached_or_timeout(robot, target_pose_deg, 2, 5)
            print("all robots have moved to the target pose.")
            for robot, mode_val in zip(robots, current_modes):
                robot.SwitchMode(mode_val)
            print("All robots are switched back to initial mode")
            
        elif key == 'h':
            current_modes = [robot.mode() for robot in robots]
            for robot in robots:
                move_j_deg(robot, HOME_POSE)
            for robot in robots:
                wait_for_reached_or_timeout(robot, HOME_POSE, 2, 5)
            print("all robots have moved to the HOME pose.")
            for robot, mode_val in zip(robots, current_modes):
                robot.SwitchMode(mode_val)
            print("All robots are switched back to initial mode")
            
        elif key == 'q':
            for robot in robots:
                robot.Stop()
            print("Program exited")
            break
        else:
            print("invalid input!")

if __name__ == "__main__":
    main()
    