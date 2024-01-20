#!/usr/bin/env python3

## this file is for traversing moving narrow window
import sys
import os
# acquire the current directory
current_dir = os.path.dirname(os.path.abspath(__file__))

# build the path to the subdirectory
subdirectory_path = os.path.join(current_dir, 'Learning_Agile')

# add to sys.path
sys.path.append(subdirectory_path)

from typing import Any
from Learning_Agile.quad_model import *
from Learning_Agile.quad_policy import *
from Learning_Agile.quad_nn import *
from Learning_Agile.quad_moving import *

# ros
import numpy as np
import rospy
from gestelt_msgs.msg import CommanderState, Goals, CommanderCommand
from geometry_msgs.msg import Pose, Accel,PoseArray,AccelStamped, Twist, PoseStamped
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Int8, Bool
import math
import time
import tf



# load the DNN2 model
FILE ="nn3_1.pth"



class MovingGate():
    def __init__(self, inputs):
        # gate status
        self.inputs=inputs
        self.gate_point0 = np.array([[-self.inputs[7]/2,0,1],[self.inputs[7]/2,0,1],[self.inputs[7]/2,0,-1],[-self.inputs[7]/2,0,-1]])
        self.gate1 = gate(self.gate_point0)
        self.gate_init_p = self.inputs[8]

        ## define the kinematics of the narrow window
        self.v =np.array([0,0.0,0.0])
        self.w = 0 #pi/2

        self.gate_move, self.V = self.gate1.move(v = self.v ,w = self.w)
    def let_gate_move(self):
        self.gate1.rotate_y(self.inputs[8])
        gate_point = self.gate1.gate_point
        self.gate1 = gate(gate_point)
        return gate_point
    
class LearningAgileAgent():
    def __init__(self) -> None:
        self.inputs = nn_sample()
        self.start_point = self.inputs[0:3]
        self.final_point = self.inputs[3:6]
        # drone state
        self.state = np.zeros(13)

        # moving gate
        self.moving_gate = MovingGate(self.inputs)
        self.gate_point = self.moving_gate.let_gate_move()

        # problem definition
        self.model = torch.load(FILE)
        self.horizon = 50

        # solver object
        self.quad1 = None
        self.quad2 = None
        

    def receive_states(self,start,end):
        """receive the start and end point, and the initial gate point, from ROS side

        Args:
            start (_type_): _description_
            end (_type_): _description_
            gate (_type_): _description_

        Returns:
            _type_: _description_
        """
        
        self.inputs[0:3]=start
        self.inputs[3:6]=end


    def problem_definition(self):
        """initial traversal problem

        Args:
            inputs (_type_): _description_

        Returns:
            _type_: _description_
        """
        self.quad1 = run_quad(goal_pos=self.inputs[3:6],ini_r=self.inputs[0:3].tolist(),ini_q=toQuaternion(self.inputs[6],[0,0,1]))
        
        self.quad1.init_obstacle(self.gate_point.reshape(12))
        self.quad1.uav1.setDyn(0.01)
 
 

    def solve_problem(self):
        ini_state = np.array(self.quad1.ini_state)
        # initializing lists for saving data
        state = self.quad1.ini_state # state= feedback from pybullet, 13-by-1, 3 position, 3 velocity (world frame), 4 quaternion, 3 angular rate
        u = [0,0,0,0]
        tm = [0,0,0,0]
        state_n = [state]
        control_n = [u]
        control_tm = [tm]
        hl_para = [0,0,0,0,0,0,0]
        hl_variable = [hl_para]
        gate_move = self.moving_gate.gate_move
        gate_n = gate(gate_move[0])
        t_guess = magni(gate_n.centroid-state[0:3])/3
        Ttra    = []
        T       = []
        Time    = []
        Pitch   = []
        j = 0

        for i in range(500):
            gate_n = gate(gate_move[i])
            t = solver(self.model,state,self.final_point,gate_n,self.moving_gate.V[i],self.moving_gate.w)
            t_tra = t+i*0.01
            gap_pitch = self.moving_gate.gate_init_p + self.moving_gate.w*i*0.01
            
            print('step',i,'tranversal time=',t,'gap_pitch=',gap_pitch*180/pi)
            # print('step',i,'abs_tranversal time=',t_tra)
            Ttra = np.concatenate((Ttra,[t_tra]),axis = 0)
            T = np.concatenate((T,[t]),axis = 0)
            Time = np.concatenate((Time,[i*0.01]),axis = 0)
            Pitch = np.concatenate((Pitch,[gap_pitch]),axis = 0)
            if (i%10)==0: # control frequency = 10 hz
            ## obtain the current gate state
            ## solve for the traversal time
                # t = solver(model,state,final_point,gate_n,V[i],w)
                # t_tra = t+i*0.01
                # print('step',i,'tranversal time=',t)
                # print('step',i,'abs_tranversal time=',t_tra)
                # Ttra = np.concatenate((Ttra,[t_tra]),axis = 0)
            #print(t,' ',i)
            ## obtain the future traversal window state
                gate_n.translate(t*self.moving_gate.V[i])
                gate_n.rotate_y(t*self.moving_gate.w)
                # print('rotation matrix I_G=',gate_n.I_G)
            ## obtain the state in window frame 
                inputs = np.zeros(18)
                inputs[16] = magni(gate_n.gate_point[0,:]-gate_n.gate_point[1,:])
                inputs[17] = atan((gate_n.gate_point[0,2]-gate_n.gate_point[1,2])/(gate_n.gate_point[0,0]-gate_n.gate_point[1,0])) # compute the actual gate pitch ange in real-time
                inputs[0:13] = gate_n.transform(state)
                inputs[13:16] = gate_n.t_final(self.final_point)
            
                out = self.model(inputs).data.numpy()
                print('tra_position=',out[0:3],'tra_time_dnn2=',out[6])
            #print(out)
                # if (horizon-1*i/10) <= 30:
                #     Horizon =30
                # else:
                #     Horizon = int(horizon-1*i/10)

            ## solve the mpc problem and get the control command
                self.quad2 = run_quad(goal_pos=inputs[13:16],horizon =50)
                u = self.quad2.get_input(inputs[0:13],u,out[0:3],out[3:6],out[6]) # control input 4-by-1 thrusts to pybullet
                j += 1
            state = np.array(self.quad1.uav1.dyn_fn(state, u)).reshape(13) # Yixiao's simulation environment ('uav1.dyn_fn'), replaced by pybullet
            state_n = np.concatenate((state_n,[state]),axis = 0)
            control_n = np.concatenate((control_n,[u]),axis = 0)
            u_m = self.quad1.uav1.u_m
            u1 = np.reshape(u,(4,1))
            tm = np.matmul(u_m,u1)
            tm = np.reshape(tm,4)
            control_tm = np.concatenate((control_tm,[tm]),axis = 0)
            hl_variable = np.concatenate((hl_variable,[out]),axis=0)
        np.save('gate_move_traj',gate_move)
        np.save('uav_traj',state_n)
        np.save('uav_ctrl',control_n)
        np.save('abs_tra_time',Ttra)
        np.save('tra_time',T)
        np.save('Time',Time)
        np.save('Pitch',Pitch)
        np.save('HL_Variable',hl_variable)
        self.quad1.uav1.play_animation(wing_len=1.5,gate_traj1=gate_move ,state_traj=state_n)
        # quad1.uav1.plot_input(control_n)
        # quad1.uav1.plot_angularrate(state_n)
        # quad1.uav1.plot_T(control_tm)
        # quad1.uav1.plot_M(control_tm)

# waypoint subscriber: start, end, gate
class LearningAgileAgentNode():

    def __init__(self):
        self.drone_state_sub = rospy.Subscriber('/mavros/local_position/pose',PoseStamped, self.drone_state_callback)
        self.waypoints_sub = rospy.Subscriber('/planner/goals_learning_agile',Goals, self.waypoints_callback)
        
        self.gate_point=np.zeros(3)
        self.end_point=np.zeros(3)
        self.start_point=np.zeros(3)
        
    def drone_state_callback(self,msg):
        """receive the drone state from ROS side

        Args:
            msg (_type_): _description_

        Returns:
            _type_: _description_
        """
       
        self.start_point = np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
        # print('start_point=',start_point)


    def waypoints_callback(self,msg):
        """receive the start and end point, and the initial gate point, from ROS side

        Args:
            msg (_type_): _description_

        Returns:
            _type_: _description_
        """
        
        self.gate_point = np.array([msg.waypoints[0].position.x,msg.waypoints[0].position.y,msg.waypoints[0].position.z])
        self.end_point = np.array([msg.waypoints[1].position.x,msg.waypoints[1].position.y,msg.waypoints[1].position.z])
        self.planning()
        
    def planning(self):


        # create the learning agile agent
        learing_agile_agent=LearningAgileAgent()
        ## receive the start and end point, and the initial gate point, from ROS side
        # rewrite the inputs
        learing_agile_agent.receive_states(start=self.start_point,end=self.end_point)

        # problem definition
        learing_agile_agent.problem_definition()

        # solve the problem
        learing_agile_agent.solve_problem()

    

# def learing_agile_agent_node():


def main():

         # ros node initialization
    learing_agile_agent_node = LearningAgileAgentNode()
    rospy.init_node('learing_agile_agent', anonymous=True)
    
    rospy.spin()




    
if __name__ == '__main__':
    main()
    

   