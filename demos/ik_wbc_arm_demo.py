import os
import sys
import inspect
import time
import numpy as np
import math
import copy

from scipy.spatial.transform import Rotation as R

from commutils.yaml_parser import load_yaml
from limbsim.env import BulletEnvWithGround
from limbsim.sim_robot_setting import SimRobotSetting
from limbsim.sim_robot_interface import SimRobotInterface
from arm_ik_wbc.ik_wbc import IkWBC
from arm_ik_wbc.trajectory_planner import TrajectoryPlanner

# ROS
import rospy
from arm_ik_wbc_ros.msg import IkWbcOut

# absolute directory of this package
rootdir = os.path.dirname(os.path.dirname(
        os.path.abspath(inspect.getfile(inspect.currentframe()))))

TIME_STEP = 0.002  # 500 Hz
MAX_TIME_SECS = 100  # maximum time to run the robot.

def main(argv):
    # Load configuration file
    if len(argv) == 1:
        cfg_file = argv[0]
    else:
        raise RuntimeError("Usage: python3 ./ik_wbc_arm_demo.py /<config file within root folder>")
        
     # Set constant control.
    configs = load_yaml(rootdir + cfg_file)
        
    #ROS init
    node_name = configs["name"] + "_WBC"
    rospy.init_node(node_name)
    send_data = rospy.Publisher("/"+node_name+"/data", IkWbcOut, queue_size=10)
    data = IkWbcOut()

    # ! Create a PyBullet simulation environment before any robots !
    env = BulletEnvWithGround(dt=TIME_STEP)  # 500 Hz

    # Create a robot instance for PyBullet.
    sim_setting = SimRobotSetting()
    sim_setting.initialize(rootdir, cfg_file)
    sim_robot = SimRobotInterface(sim_setting)

    # Add the robot to the env to update the internal structure of the robot.
    env.add_robot(sim_robot)
    
    # fetch robot params
    joint_des_pos = np.array(configs["control_variables"]["joint_des_pos"])
    joint_names = configs["control_variables"]["joint_names"]
    arm_urdf_path = rootdir + configs["sim_robot_variables"]["urdf_filename"]
    arm_mesh_path = rootdir + configs["sim_robot_variables"]["mesh_foldername"]
    EE_frame_name = configs["sim_robot_variables"]["limb_endeff_frame_names"]
    EE_joint_name = configs["sim_robot_variables"]["limb_endeff_joint_names"]
    base_frame_name = configs["sim_robot_variables"]["base_name"]
    
    # Initialise ik wbc
    controller = IkWBC(arm_urdf_path, arm_mesh_path, EE_frame_name, EE_frame_name, base_frame_name, joint_des_pos)
    
    # Initialise targect dict
    EE_target_pos = [0]
    EE_target_ori = [0]
    for i in range(len(controller.EE_index_list_frame)):
        EE_target_pos[i] = controller.EE_frame_pos[i].reshape(3,1)
        rot_mat = R.from_matrix(controller.EE_frame_ori[i])
        rot_euler = rot_mat.as_euler('xyz')
        EE_target_ori[i] = rot_euler.reshape(3,1)
    target_dict = {"task name":[], "target pos":[], "target ori":[]}
    target_dict["task name"] = EE_frame_name
    target_dict["target pos"] = copy.deepcopy(EE_target_pos)
    target_dict["target ori"] = copy.deepcopy(EE_target_ori)
    
    # Initialise planner
    planner = TrajectoryPlanner(copy.deepcopy(target_dict))
    
    start_time = env.get_time_since_start()
    current_time = start_time
    
    # DRAW A SQUARE
    while current_time - start_time < MAX_TIME_SECS:
        #0
        end_target_dict = copy.deepcopy(target_dict)
        end_target_dict["target pos"][0][0] = end_target_dict["target pos"][0][0] + 0
        interval, traj_interval = planner.generate_trajectory(target_dict, end_target_dict)
        for i in traj_interval:
            # Fetch new target dict
            target_dict = planner.run_stored_trajectory(target_dict, i)
            #print(target_dict)
            
            # Fetch time
            start_time_env = current_time
            start_time_wall = time.time()
            
            # Run WBC
            joint_des_pos = controller.runWBC(target_dict)

            # Apply joint positions to robot
            sim_robot.apply_joint_positions(joint_names, joint_des_pos)

            # Step the simulation environment
            env.step(sleep=False)
            
            # Send data
            data.ik_wbc_out = joint_des_pos
            send_data.publish(data)

            current_time = env.get_time_since_start()
            # Add sleep time
            expected_duration = current_time - start_time_env
            actual_duration = time.time() - start_time_wall
            if actual_duration < expected_duration:
                time.sleep(expected_duration - actual_duration)
        #1
        end_target_dict = copy.deepcopy(target_dict)
        end_target_dict["target pos"][0][1] = end_target_dict["target pos"][0][1] + 0.2
        end_target_dict["target ori"][0][2] = end_target_dict["target ori"][0][2] + math.pi/8
        interval, traj_interval = planner.generate_trajectory(target_dict, end_target_dict)
        for i in traj_interval:
            # Fetch new target dict
            target_dict = planner.run_stored_trajectory(target_dict, i)
            #print(target_dict)
            
            # Fetch time
            start_time_env = current_time
            start_time_wall = time.time()
            
            # Run WBC
            joint_des_pos = controller.runWBC(target_dict)

            # Apply joint positions to robot
            sim_robot.apply_joint_positions(joint_names, joint_des_pos)

            # Step the simulation environment
            env.step(sleep=False)

            # Send data
            data.ik_wbc_out = joint_des_pos
            send_data.publish(data)

            current_time = env.get_time_since_start()
            # Add sleep time
            expected_duration = current_time - start_time_env
            actual_duration = time.time() - start_time_wall
            if actual_duration < expected_duration:
                time.sleep(expected_duration - actual_duration)
        
        #2
        end_target_dict = copy.deepcopy(target_dict)
        end_target_dict["target pos"][0][2] = end_target_dict["target pos"][0][2] + 0.1
        interval, traj_interval = planner.generate_trajectory(target_dict, end_target_dict)
        for i in traj_interval:
            # Fetch new target dict
            target_dict = planner.run_stored_trajectory(target_dict, i)
            #print(target_dict)
            
            # Fetch time
            start_time_env = current_time
            start_time_wall = time.time()
            
            # Run WBC
            joint_des_pos = controller.runWBC(target_dict)

            # Apply joint positions to robot
            sim_robot.apply_joint_positions(joint_names, joint_des_pos)

            # Step the simulation environment
            env.step(sleep=False)
            
            # Send data
            data.ik_wbc_out = joint_des_pos
            send_data.publish(data)

            current_time = env.get_time_since_start()
            # Add sleep time
            expected_duration = current_time - start_time_env
            actual_duration = time.time() - start_time_wall
            if actual_duration < expected_duration:
                time.sleep(expected_duration - actual_duration)
                
        #3
        end_target_dict = copy.deepcopy(target_dict)
        end_target_dict["target pos"][0][1] = end_target_dict["target pos"][0][1] -0.4
        end_target_dict["target ori"][0][2] = end_target_dict["target ori"][0][2] - math.pi/4
        interval, traj_interval = planner.generate_trajectory(target_dict, end_target_dict)
        for i in traj_interval:
            # Fetch new target dict
            target_dict = planner.run_stored_trajectory(target_dict, i)
            #print(target_dict)
            
            # Fetch time
            start_time_env = current_time
            start_time_wall = time.time()
            
            # Run WBC
            joint_des_pos = controller.runWBC(target_dict)

            # Apply joint positions to robot
            sim_robot.apply_joint_positions(joint_names, joint_des_pos)

            # Step the simulation environment
            env.step(sleep=False)
            
            # Send data
            data.ik_wbc_out = joint_des_pos
            send_data.publish(data)
            
            current_time = env.get_time_since_start()
            # Add sleep time
            expected_duration = current_time - start_time_env
            actual_duration = time.time() - start_time_wall
            if actual_duration < expected_duration:
                time.sleep(expected_duration - actual_duration)
                   
        #4
        end_target_dict = copy.deepcopy(target_dict)
        end_target_dict["target pos"][0][2] = end_target_dict["target pos"][0][2] - 0.1
        interval, traj_interval = planner.generate_trajectory(target_dict, end_target_dict)
        for i in traj_interval:
            # Fetch new target dict
            target_dict = planner.run_stored_trajectory(target_dict, i)
            #print(target_dict)
            
            # Fetch time
            start_time_env = current_time
            start_time_wall = time.time()
            
            # Run WBC
            joint_des_pos = controller.runWBC(target_dict)

            # Apply joint positions to robot
            sim_robot.apply_joint_positions(joint_names, joint_des_pos)

            # Step the simulation environment
            env.step(sleep=False)

            # Send data
            data.ik_wbc_out = joint_des_pos
            send_data.publish(data)

            current_time = env.get_time_since_start()
            # Add sleep time
            expected_duration = current_time - start_time_env
            actual_duration = time.time() - start_time_wall
            if actual_duration < expected_duration:
                time.sleep(expected_duration - actual_duration)
                
        #5
        interval, traj_interval = planner.generate_homing_trajectory(target_dict)
        for i in traj_interval:
            # Fetch new target dict
            target_dict = planner.run_stored_trajectory(target_dict, i)
            #print(target_dict)
            
            # Fetch time
            start_time_env = current_time
            start_time_wall = time.time()
            
            # Run WBC
            joint_des_pos = controller.runWBC(target_dict)

            # Apply joint positions to robot
            sim_robot.apply_joint_positions(joint_names, joint_des_pos)

            # Step the simulation environment
            env.step(sleep=False)

            # Send data
            data.ik_wbc_out = joint_des_pos
            send_data.publish(data)

            current_time = env.get_time_since_start()
            # Add sleep time
            expected_duration = current_time - start_time_env
            actual_duration = time.time() - start_time_wall
            if actual_duration < expected_duration:
                time.sleep(expected_duration - actual_duration)

if __name__ == "__main__":
    main(sys.argv[1:])
