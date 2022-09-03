#!/usr/bin/env python3.8
import os
import sys
import inspect
import time
import numpy as np
import math
import copy

from scipy.spatial.transform import Rotation as R

from commutils.yaml_parser import load_yaml
from arm_ik_wbc.ik_wbc import IkWBC
from arm_ik_wbc.trajectory_planner import TrajectoryPlanner

# ROS
import rospy
from interbotix_ihrc.msg import JointGroupCommand
from sensor_msgs.msg import JointState
from arm_ik_wbc_ros.msg import IkWbcIn

# absolute directory of this package
rootdir = os.path.dirname(os.path.dirname(
        os.path.abspath(inspect.getfile(inspect.currentframe()))))
        
split_rootdir = rootdir.split("install")
rootdir = split_rootdir[0]
rootdir = rootdir + "src/arm_ik_wbc_ros"

TIME_STEP = 0.002  # 500 Hz
MAX_TIME_SECS = 100  # maximum time to run the robot.

def input_callback(input_ref):
    global x_ref
    global y_ref
    global z_ref
    global roll_ref
    global pitch_ref
    global yaw_ref
    global input_init
    input_init = True
    
    try:
        x_ref = input_ref.ik_wbc_in[0]
        y_ref = input_ref.ik_wbc_in[1]
        z_ref = input_ref.ik_wbc_in[2]
        roll_ref = input_ref.ik_wbc_in[3]
        pitch_ref = input_ref.ik_wbc_in[4]
        yaw_ref = input_ref.ik_wbc_in[5]
    except:
        x_ref = target_dict["target pos"][0][0]
        y_ref = target_dict["target pos"][0][1]
        z_ref = target_dict["target pos"][0][2]
        roll_ref = target_dict["target ori"][0][0]
        pitch_ref = target_dict["target ori"][0][1]
        yaw_ref = target_dict["target ori"][0][2]

def joint_callback(joint_state_data):
    global joint_states
    global arm_init
    arm_init = True
    joint_states = []
    for i in range(len(joint_state_data.position)):
        joint_states.append(joint_state_data.position[i])

def homing(publisher, home_state):
    current_state = np.array(joint_states)
    current_state = [current_state[0],current_state[1],current_state[3],current_state[5],current_state[6],current_state[7]]
    homing_traj = []
    duration = 5
    send_cmd = JointGroupCommand()
    send_cmd.name = 'arm'
    for i in range(len(home_state)):
        traj = np.linspace(current_state[i], home_state[i], num=int(duration/TIME_STEP))
        homing_traj.append(traj)
    for i in range(int(duration/TIME_STEP)):
        start_time = time.time()
        cmd = []
        for ii in range(len(home_state)):
            current_state[ii] = homing_traj[ii][i]
        send_cmd.cmd = current_state
        publisher.publish(send_cmd)
        #print(current_state)
        while (time.time() - start_time < TIME_STEP):
            pass
    
def main():
    global target_dict
    global arm_init
    global input_init
    # Load configuration file
    try:
        cfg_file = rospy.get_param("/interbotix_arm_wbc/config")#argv[0]
    except:
        raise RuntimeError("Usage: python3 ./ik_wbc_arm_demo.py /<config file within root folder>")
        
     # Set constant control.
    configs = load_yaml(rootdir + cfg_file)
        
    #ROS init
    node_name = configs["name"] + "_WBC"
    rospy.init_node(node_name)
    rospy.Subscriber("/"+node_name+"/input", IkWbcIn, input_callback)
    rospy.Subscriber("/"+configs["name"]+"/joint_states", JointState, joint_callback)
    send_data = rospy.Publisher("/"+configs["name"]+"/commands/joint_group", JointGroupCommand, queue_size=10)
    data = JointGroupCommand()
    data.name = 'arm'
    arm_init = False
    input_init = False
    
    # fetch robot params
    joint_des_pos = np.array(configs["control_variables"]["joint_des_pos"])
    joint_names = configs["control_variables"]["joint_names"]
    arm_urdf_path = rootdir + configs["sim_robot_variables"]["urdf_filename"]
    arm_mesh_path = rootdir + configs["sim_robot_variables"]["mesh_foldername"]
    EE_frame_name = configs["sim_robot_variables"]["limb_endeff_frame_names"]
    EE_joint_name = configs["sim_robot_variables"]["limb_endeff_joint_names"]
    base_frame_name = configs["sim_robot_variables"]["base_name"]
    
    #Init robot
    while arm_init == False:
        pass
    homing(send_data, joint_des_pos)
    
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
    
    while input_init == False:
        pass
    
    while True:

        # Listen for wbc reference command and update target_dict
        target_dict["target pos"][0][0] = x_ref
        target_dict["target pos"][0][1] = y_ref
        target_dict["target pos"][0][2] = z_ref
        target_dict["target ori"][0][0] = roll_ref
        target_dict["target ori"][0][1] = pitch_ref
        target_dict["target ori"][0][2] = yaw_ref

        joint_des_pos = controller.runWBC(target_dict)

        # Send data
        data.cmd = joint_des_pos
        send_data.publish(data)


if __name__ == "__main__":
    main()
