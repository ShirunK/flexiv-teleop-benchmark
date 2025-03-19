import time
import flexivrdk
import csv
import os
import ast
import math

PI = 3.141592653

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

robot = flexivrdk.Robot('Rizon4s-062242')
mode = flexivrdk.Mode
filename = "save_pose.csv"

if not os.path.exists(filename):
    with open(filename, mode="w", newline="") as file:
        csv_writer = csv.writer(file)
        csv_writer.writerow(["joint_positions_deg", "joint_positions_rad", "tcp_pose_quat", "tcp_pose_euler"])
    print("Created new file:", filename)

while True:
    print("\nPress 's' to save the current pose, 'g' to go to a saved pose, 'h' to go home, or 'q' to quit.")
    key = input("Enter your choice: ").strip().lower()
    
    if key == 's':
        save_pose_rad = robot.states().q
        save_pose_deg = [angle * 180 / PI for angle in save_pose_rad]
        current_pose_tcp = robot.states().tcp_pose.copy()

        tcp_pose_quat = current_pose_tcp
        x, y, z = current_pose_tcp[0:3]
        qw, qx, qy, qz = current_pose_tcp[3:]
        euler_angles = quaternion_to_euler(qw, qx, qy, qz)
        tcp_pose_euler = [x, y, z] + euler_angles
        
        print("Current Joint Position (rad):", save_pose_rad)
        print("Current Joint Position (deg):", save_pose_deg)
        print("Current TCP pose (quat):", tcp_pose_quat)
        print("Current TCP pose (euler deg):", tcp_pose_euler)
        
        with open(filename, mode="a", newline="") as file:
            csv_writer = csv.writer(file)
            csv_writer.writerow([str(save_pose_deg), str(save_pose_rad), str(tcp_pose_quat), str(tcp_pose_euler)])
        print("Pose saved to", filename)
        
    elif key == 'g':
        poses_deg = []
        poses_rad = []
        tcp_quats = []
        tcp_eulers = []
        with open(filename, mode="r", newline="") as file:
            csv_reader = csv.reader(file)
            header = next(csv_reader, None)  # skip the csv head
            for row in csv_reader:
                try:
                    pose_deg = ast.literal_eval(row[0])
                    pose_rad = ast.literal_eval(row[1])
                    tcp_quat = ast.literal_eval(row[2])
                    tcp_euler = ast.literal_eval(row[3])
                except Exception as e:
                    print("Error parsing row:", row, e)
                    continue
                poses_deg.append(pose_deg)
                poses_rad.append(pose_rad)
                tcp_quats.append(tcp_quat)
                tcp_eulers.append(tcp_euler)
        
        if not poses_deg:
            print("No poses found in", filename)
            continue
        
        print("Saved poses:")
        for idx, pose in enumerate(poses_deg):
            print(f"{idx}: {pose} | TCP (quat): {tcp_quats[idx]} | TCP (euler): {tcp_eulers[idx]}")
        
        try:
            index = int(input("Enter the pose index to go to: ").strip())
            if index < 0 or index >= len(poses_deg):
                print("Invalid index selected.")
                continue
        except ValueError:
            print("Invalid input. Please enter a valid integer index.")
            continue
        
        target_pose_deg = poses_deg[index]
        print("Moving to pose (deg):", target_pose_deg)
        
        robot.SwitchMode(mode.NRT_PRIMITIVE_EXECUTION)
        robot.ExecutePrimitive(
            "MoveJ",
            {
                "target": flexivrdk.JPos(target_pose_deg, [0, 0, 0, 0, 0, 0])
            }
        )
        while not robot.primitive_states()["reachedTarget"]:
            time.sleep(1)
        print("Reached target pose.")
    elif key == 'h':
        robot.SwitchMode(mode.NRT_PRIMITIVE_EXECUTION)
        robot.ExecutePrimitive("Home", dict())
        while not robot.primitive_states()["reachedTarget"]:
            time.sleep(1)
        print("Robot homed.")
    elif key == 'q':
        robot.Stop()
        print("Exiting program.")
        break
    else:
        print("Invalid input. Please try again.")
