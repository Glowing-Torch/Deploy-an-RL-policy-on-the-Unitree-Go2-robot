#!/usr/bin/env python3
import rclpy
import torch
from config import Config
import numpy as np
import os
import sys
from rclpy.node import Node
import argparse
from pathlib import Path
from unitree_go.msg import LowState
from wireless_controller_command import Controller
from std_msgs.msg import Float32MultiArray
project_root=Path(__file__).parents[4]

class dataReciever(Node):
    def __init__(self,config:Config):
        super().__init__("data_reciever")
        self.config = config
        self.cmd_sub=Controller(self)
        # Initialize the policy network()
        script_dir = os.path.dirname(os.path.abspath(__file__))  
        policy_path = os.path.join(script_dir, self.config.policy_path)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        # print(self.device)
        self.policy = torch.jit.load(policy_path)
        self.policy.to(self.device)
        self.policy.eval()
        # Initializing process variables
        self.qj = np.zeros(config.num_actions, dtype=np.float32)
        self.dqj = np.zeros(config.num_actions, dtype=np.float32)
        self.action = np.zeros(config.num_actions, dtype=np.float32)
        self.target_dof_pos = config.default_angles.copy()
        self.obs = np.zeros(config.num_obs, dtype=np.float32)
        self.cur_obs=np.zeros(45,dtype=np.float32)
        self.cmd = np.array([0.0, 0, 0])
        self.low_state=LowState()

        if args.simulation:
            self.low_state_sub=self.create_subscription(LowState,"/mujoco/lowstate",self.low_state_callback,10)
            print("reading data from simuation")
        else:    
            self.low_state_sub=self.create_subscription(LowState,"/lowstate",self.low_state_callback,10) #500 HZ
            print("reading data from reality")


        self.get_logger().info("Waiting for data")
        self.timer = self.create_timer(0.02, self.run)
        self.target_pos_puber=self.create_publisher(Float32MultiArray,"/rl/target_pos",10)



    def low_state_callback(self,msg:LowState):
        self.low_state=msg
    
    def run(self):
        # self.get_logger().info("running")
        # Get the current joint position and velocity
        if (self.cmd_sub.keys==48):
            sys.exit()
            
        for i in range(12):
            self.qj[i] = self.low_state.motor_state[i].q
            self.dqj[i] = self.low_state.motor_state[i].dq
        sequence = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]
        
        # comment these two lines for simulation
        self.qj=self.qj[sequence]
        self.dqj=self.dqj[sequence]
        
        # imu_state quaternion: w, x, y, z
        quat = self.low_state.imu_state.quaternion
        ang_vel = np.array(self.low_state.imu_state.gyroscope, dtype=np.float32)

        # create observation
        gravity_orientation = self.get_gravity_orientation(quat)
        qj_obs = (self.qj - self.config.default_angles) * self.config.dof_pos_scale
        dqj_obs = self.dqj * self.config.dof_vel_scale

        ang_vel = ang_vel * self.config.ang_vel_scale
        self.cmd=np.zeros(3)
        if self.cmd_sub.keys==3:
            self.cmd_sub.linear_x,self.cmd_sub.linear_y=self.cmd_sub.get_left_stick() *self.cmd_sub.vel_scale
            self.cmd_sub.angular_z=self.cmd_sub.get_right_stick()*self.cmd_sub.vel_scale

        else:
            self.cmd_sub.linear_x+=-np.sign(self.cmd_sub.linear_x)*0.02
            self.cmd_sub.linear_y+=-np.sign(self.cmd_sub.linear_y)*0.02
            self.cmd_sub.angular_z+=-np.sign(self.cmd_sub.angular_z)*0.02

        self.cmd=np.array([self.cmd_sub.linear_x,self.cmd_sub.linear_y,self.cmd_sub.angular_z])
        print(self.cmd)
        self.cur_obs[:3] = self.cmd * self.config.cmd_scale 
        self.cur_obs[3:6] = gravity_orientation
        self.cur_obs[6:9] = ang_vel
        self.cur_obs[9: 21] = qj_obs
        self.cur_obs[21:33] = dqj_obs
        self.cur_obs[33:45] = self.action
        self.obs=np.concatenate((self.obs[45:],self.cur_obs[:45]))
        self.obs=np.clip(self.obs,-100,100)
        # Get the action from the policy network
        obs_tensor = torch.from_numpy(self.obs).unsqueeze(0).to(self.device)
        with torch.no_grad():
            action_tensor = self.policy(obs_tensor)
        self.action = action_tensor.cpu().numpy().squeeze()

        # self.action=np.clip(self.action,-100,100)
        
        # transform action to target_dof_pos
        self.target_dof_pos = self.config.default_angles + self.action * self.config.action_scale
        #tau computed by network in the sequence of: 
        ''' 
        FL_hip,FL_thigh,FL_calf
        FR_hip,FR_thigh,FR_calf
        RL_hip,RL_thigh,RL_calf
        RR_hip,RR_thigh,RR_calf
        '''
        # The true order of the actuator in /lowcmd (also in mujoco simulator) should be:
        ''' 
        FR_hip,FR_thigh,FR_calf
        FL_hip,FL_thigh,FL_calf
        RR_hip,RR_thigh,RR_calf
        RL_hip,RL_thigh,RL_calf
        '''
        msg=Float32MultiArray()
        msg.data.extend(self.target_dof_pos.astype(np.float32)[sequence].tolist())
        self.target_pos_puber.publish(msg)
        
        
        
    @staticmethod
    def get_gravity_orientation(quaternion):
        qw = quaternion[0]
        qx = quaternion[1]
        qy = quaternion[2]
        qz = quaternion[3]

        gravity_orientation = np.zeros(3)

        gravity_orientation[0] = 2 * (-qz * qx + qw * qy)
        gravity_orientation[1] = -2 * (qz * qy + qw * qx)
        gravity_orientation[2] = 1 - 2 * (qw * qw + qz * qz)
        return gravity_orientation

    
def main():
    rclpy.init()
    reciever_node=dataReciever(config=config)
    rclpy.spin(reciever_node)
    reciever_node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    # Load config
    config_path = project_root/"src"/"deploy_rl_policy"/"configs"/"go2.yaml"
    config = Config(config_path)
    parser = argparse.ArgumentParser()
    parser.add_argument('--is_simulation', type=str, choices=["True", "False"], default="True")
    args = parser.parse_args()
    args.simulation = args.is_simulation == "True"
    main()