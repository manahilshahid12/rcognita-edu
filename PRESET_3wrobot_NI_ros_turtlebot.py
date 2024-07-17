"""
Preset: a 3-wheel robot (kinematic model a. k. a. non-holonomic integrator).

"""
  
import pathlib  
  
import warnings
import csv
from datetime import datetime
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np

import systems
import simulator
import controllers
import loggers
import visuals
from utilities import on_key_press

import argparse

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from tf.transformations import euler_from_quaternion
import tf.transformations as tftr 
import math
import threading


class ROS_preset:

    def __init__(self, ctrl_mode, state_goal, state_init, my_ctrl_nominal, my_sys, my_ctrl_benchmarking, my_logger=None, datafile=None):
        
        self.RATE = rospy.get_param('/rate', 20)
        self.lock = threading.Lock()
        
        #Initialization
        self.state_init = state_init
        self.state_goal = state_goal
        self.system = my_sys
        
        self.ctrl_nominal = my_ctrl_nominal
        self.ctrl_benchm = my_ctrl_benchmarking
        
        self.dt = 0.0
        self.time_start = 0.0
        
        # Topics
        
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=False)
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.odometry_callback)
        
        self.state = np.zeros(3)
        self.dstate = np.zeros(3)
        self.new_state = np.zeros(3)
        self.new_dstate = np.zeros(3)
        
        self.datafiles = datafiles
        self.logger = my_logger
        self.ctrl_mode = ctrl_mode
        
        self.rotation_counter = 0
        self.prev_theta = 0
        self.new_theta = 0
        
        #theta_goal = self.state_goal[2]
        
        self.ctrl_bnds = np.array([[-0.22, 0.22],  #linear velocity
                                   [-2, 2]]) #angular velocity
        # Complete Rotation Matrix
        self.rotation_matrix = np.zeros((3, 3))  # here
        
    def odometry_callback(self, msg):
    
        self.lock.acquire()
        
        # Read current robot state
        x = msg.pose.pose.position.x
        # Complete for y and orientation
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
          
        # Transform quat2euler using tf transformations: complete!
        current_rpy = tftr.euler_from_quaternion([q.x, q.y, q.z, q.w]) 
        
        # Extract Theta (orientation about Z)
        theta = current_rpy[2]
        
        self.state = [x, y, theta]
        
        # Make transform matrix from 'robot body' frame to 'goal' frame
        theta_goal = self.state_goal[2]
        
        # Complete rotation matrix
        rotation_matrix = np.array([[np.cos(theta_goal), -np.sin(theta_goal), 0], 
                                    [np.sin(theta_goal), np.cos(theta_goal), 0], 
                                    [0, 0, 1]])
        
        state_matrix = np.vstack([self.state_goal[0], self.state_goal[1], 0]) # [x, y, 0] -- column   
        
        # Compute Transformation matrix 
        
        self.t_matrix =np.block([[rotation_matrix, state_matrix],
                           [np.array([0, 0, 0, 1])]]) # Complete
        
        # Complete rotation counter for turtlebot3
        
        ''' Your solution goes here (rotation_counter) '''
        if math.copysign(1, self.prev_theta) != math.copysign(1, theta) and abs(self.prev_theta) > np.pi:
            if math.copysign(1, self.prev_theta) == -1:
                self.rotation_counter -= 1

            else:
                self.rotation_counter +=1



        self.prev_theta = theta
        theta = theta + 2 * np.pi * self.rotation_counter
        self.new_theta = theta
        
        # Orientation transform
        new_theta = theta - theta_goal
        
        # Do position transform
        
        ''' 
        Your solution goes here 
        self.new_state = using tranformations :) Have fun!
        '''
        temp = np.array([x, y, 0, 1])
        self.new_state = np.linalg.inv(self.t_matrix)@temp.T
        self.new_state = [self.new_state[0], self.new_state[1], new_theta]


        self.lock.release()
        
    def spin(self, is_print_sim_step=False, is_log_data=False):
    
        rospy.loginfo('ROS-Preset has been activated!')
        #start_time = time_lib.time()
        rate = rospy.Rate(self.RATE)
        self.time_start = rospy.get_time()
        
        while not rospy.is_shutdown():
        
            t = rospy.get_time() - self.time_start
            self.t = t
            
            velocity=Twist()
            
            action = controllers.ctrl_selector(t,
                                               self.new_state,
                                               self.ctrl_nominal,
                                               None, 
                                               self.ctrl_benchm, 
                                               self.ctrl_mode)
            
            self.system.receive_action(action)
            self.ctrl_benchm.receive_sys_state(self.system._state)
            self.ctrl_benchm.upd_accum_obj(self.new_state, action)

            

            for k in range(2):
                action[k] = np.clip(action[k], self.ctrl_bnds[k, 0], self.ctrl_bnds[k, 1])
            
            xCoord = self.new_state[0]
            yCoord = self.new_state[1]
            alpha = self.new_state[2]
        
            run_obj = self.ctrl_benchm.run_obj(self.new_state, action)
            accum_obj = self.ctrl_benchm.accum_obj_val

            if is_print_sim_step:
                
                self.logger.print_sim_step(t, xCoord, yCoord, alpha, run_obj, accum_obj, action)
            
            if is_log_data:
                self.logger.log_data_row(datafiles[0], t, xCoord, yCoord, alpha, run_obj, accum_obj, action)
                print(f"Logged data - t: {t}, x: {xCoord}, y: {yCoord}, alpha: {alpha}, run_obj: {run_obj}, accum_obj: {accum_obj}, action: {action}")    
            
            #self.ctrl_benchm.receive_sys_state(self.new_state)
            
            
           # Generate ROSmsg from action
        
            velocity.linear.x = action[0]
            velocity.angular.z = action[1]
            self.pub_cmd_vel.publish(velocity) #publishing the velocity in the gazebo enviornment by sending info to the bot
        

            rate.sleep()
           
        rospy.loginfo('Task completed or interrupted!')
       
if __name__ == "__main__":
    rospy.init_node('ros_preset_node')
#----------------------------------------Set up dimensions
dim_state = 3
dim_input = 2
dim_output = dim_state
dim_disturb = 0

dim_R1 = dim_output + dim_input
dim_R2 = dim_R1

description = "Agent-environment preset: a 3-wheel robot (kinematic model a.k.a. non-holonomic integrator)."

parser = argparse.ArgumentParser(description=description)

parser.add_argument('--ctrl_mode', metavar='ctrl_mode', type=str,
                    choices=['MPC',
                             "N_CTRL","Stanley_CTRL"],
                    default='N_CTRL',
                    help='Control mode. Currently available: ' +
                    '----manual: manual constant control specified by action_manual; ' +
                    '----nominal: nominal controller, usually used to benchmark optimal controllers;' +                     
                    '----MPC:model-predictive control; ' +
                    '----RQL: Q-learning actor-critic with Nactor-1 roll-outs of running objective; ' +
                    '----SQL: stacked Q-learning; ' + 
                    '----RLStabLyap: (experimental!) learning agent with Lyapunov-like stabilizing contraints.')
parser.add_argument('--dt', type=float, metavar='dt',
                    default=0.1,
                    help='Controller sampling time.' )
parser.add_argument('--t1', type=float, metavar='t1',
                    default=30,
                    help='Final time of episode.' )
parser.add_argument('--Nruns', type=int,
                    default=1,
                    help='Number of episodes. Learned parameters are not reset after an episode.')
parser.add_argument('--is_log_data', type=int,
                    default=1,
                    help='Flag to log data into a data file. Data are stored in simdata folder.')
parser.add_argument('--is_visualization', type=int,
                    default=1,
                    help='Flag to produce graphical output.')
parser.add_argument('--is_print_sim_step', type=int,
                    default=1,
                    help='Flag to print simulation data into terminal.')
parser.add_argument('--action_manual', type=float,
                    default=[-5, -3], nargs='+',
                    help='Manual control action to be fed constant, system-specific!')
parser.add_argument('--Nactor', type=int,
                    default=6,
                    help='Horizon length (in steps) for predictive controllers.')
parser.add_argument('--pred_step_size_multiplier', type=float,
                    default=5.0,
                    help='Size of each prediction step in seconds is a pred_step_size_multiplier multiple of controller sampling time dt.')
parser.add_argument('--buffer_size', type=int,
                    default=25,
                    help='Size of the buffer (experience replay) for model estimation, agent learning etc.')
parser.add_argument('--run_obj_struct', type=str,
                    default='quadratic',
                    choices=['quadratic',
                             'biquadratic'],
                    help='Structure of running objective function.')
parser.add_argument('--R1_diag', type=float, nargs='+',
                    default=[100, 100, 10, 0, 0],
                    help='Parameter of running objective function. Must have proper dimension. ' +
                    'Say, if chi = [observation, action], then a quadratic running objective reads chi.T diag(R1) chi, where diag() is transformation of a vector to a diagonal matrix.')
parser.add_argument('--R2_diag', type=float, nargs='+',
                    default=[1, 10, 1, 0, 0],
                    help='Parameter of running objective function . Must have proper dimension. ' + 
                    'Say, if chi = [observation, action], then a bi-quadratic running objective reads chi**2.T diag(R2) chi**2 + chi.T diag(R1) chi, ' +
                    'where diag() is transformation of a vector to a diagonal matrix.')
parser.add_argument('--Ncritic', type=int,
                    default=25,
                    help='Critic stack size (number of temporal difference terms in critic cost).')
parser.add_argument('--gamma', type=float,
                    default=0.9,
                    help='Discount factor.')
parser.add_argument('--critic_period_multiplier', type=float,
                    default=1.0,
                    help='Critic is updated every critic_period_multiplier times dt seconds.')
parser.add_argument('--critic_struct', type=str,
                    default='quad-mix', choices=['quad-lin',
                                                   'quadratic',
                                                   'quad-nomix',
                                                   'quad-mix',
                                                   'poly3',
                                                   'poly4'],
                    help='Feature structure (critic). Currently available: ' +
                    '----quad-lin: quadratic-linear; ' +
                    '----quadratic: quadratic; ' +
                    '----quad-nomix: quadratic, no mixed terms; ' +
                    '----quad-mix: quadratic, mixed observation-action terms (for, say, Q or advantage function approximations); ' +
                    '----poly3: 3-order model, see the code for the exact structure; ' +
                    '----poly4: 4-order model, see the code for the exact structure. '
                    )
parser.add_argument('--actor_struct', type=str,
                    default='quad-nomix', choices=['quad-lin',
                                                   'quadratic',
                                                   'quad-nomix'],
                    help='Feature structure (actor). Currently available: ' +
                    '----quad-lin: quadratic-linear; ' +
                    '----quadratic: quadratic; ' +
                    '----quad-nomix: quadratic, no mixed terms.')
parser.add_argument('--init_robot_pose_x', type=float,
                    default=-10.0,
                    help='Initial x-coordinate of the robot pose.')
parser.add_argument('--init_robot_pose_y', type=float,
                    default=-10.0,
                    help='Initial y-coordinate of the robot pose.')
parser.add_argument('--init_robot_pose_theta', type=float,
                    default=2.6,
                    help='Initial orientation angle (in radians) of the robot pose.')
parser.add_argument('--distortion_x', type=float,
                    default=-0.6,
                    help='X-coordinate of the center of distortion.')
parser.add_argument('--distortion_y', type=float,
                    default=-0.5,
                    help='Y-coordinate of the center of distortion.')
parser.add_argument('--distortion_sigma', type=float,
                    default=0.1,
                    help='Standard deviation (sigma) of distortion.')
parser.add_argument('--seed', type=int,
                    default=1,
                    help='Seed for random number generation.')

args = parser.parse_args()

seed=args.seed
print(seed)

xdistortion_x = args.distortion_x
ydistortion_y = args.distortion_y
distortion_sigma = args.distortion_sigma

x = args.init_robot_pose_x
y = args.init_robot_pose_y
theta = args.init_robot_pose_theta

while theta > np.pi:
        theta -= 2 * np.pi
while theta < -np.pi:
        theta += 2 * np.pi

state_init = np.array([x, y, theta])

args.action_manual = np.array(args.action_manual)

pred_step_size = args.dt *  args.pred_step_size_multiplier
critic_period = args.dt * args.critic_period_multiplier

R1 = np.diag(np.array(args.R1_diag))
R2 = np.diag(np.array(args.R2_diag))

assert args.t1 > args.dt > 0.0
assert state_init.size == dim_state

globals().update(vars(args))

#----------------------------------------Fixed settings
is_disturb = 0
is_dyn_ctrl = 0

t0 = 0

action_init = 0 * np.ones(dim_input)

# Solver
atol = 1e-3
rtol = 1e-2

# xy-plane
xMin = -4#-1.2
xMax = 0.2
yMin = -4#-1.2
yMax = 0.2

# Control constraints
v_min = -0.22 *10
v_max = 0.22 *10
omega_min = -2.84
omega_max = 2.84

ctrl_bnds=np.array([[v_min, v_max], [omega_min, omega_max]])

#----------------------------------------Initialization : : system
my_sys = systems.Sys3WRobotNI(sys_type="diff_eqn", 
                                     dim_state=dim_state,
                                     dim_input=dim_input,
                                     dim_output=dim_output,
                                     dim_disturb=dim_disturb,
                                     pars=[],
                                     ctrl_bnds=ctrl_bnds,
                                     is_dyn_ctrl=is_dyn_ctrl,
                                     is_disturb=is_disturb,
                                     pars_disturb=[])

observation_init = my_sys.out(state_init)

xCoord0 = state_init[0]
yCoord0 = state_init[1]
alpha0 = state_init[2]
alpha_deg_0 = alpha0/2/np.pi

#----------------------------------------Initialization : : model

#----------------------------------------Initialization : : controller
my_ctrl_nominal = None 

# Predictive optimal controller
my_ctrl_opt_pred = controllers.ControllerOptimalPredictive(dim_input,
                                           dim_output,
                                           ctrl_mode,
                                           ctrl_bnds = ctrl_bnds,
                                           action_init = [],
                                           t0 = t0,
                                           sampling_time = dt,
                                           Nactor = Nactor,
                                           pred_step_size = pred_step_size,
                                           sys_rhs = my_sys._state_dyn,
                                           sys_out = my_sys.out,
                                           state_sys = state_init,
                                           buffer_size = buffer_size,
                                           gamma = gamma,
                                           Ncritic = Ncritic,
                                           critic_period = critic_period,
                                           critic_struct = critic_struct,
                                           run_obj_struct = run_obj_struct,
                                           run_obj_pars = [R1],
                                           observation_target = [],
                                           state_init=state_init,
                                           obstacle=[xdistortion_x, ydistortion_y,distortion_sigma],
                                           seed=seed)


my_ctrl_benchm = my_ctrl_opt_pred
    
#----------------------------------------Initialization : : simulator
my_simulator = simulator.Simulator(sys_type = "diff_eqn",
                                   closed_loop_rhs = my_sys.closed_loop_rhs,
                                   sys_out = my_sys.out,
                                   state_init = state_init,
                                   disturb_init = [],
                                   action_init = action_init,
                                   t0 = t0,
                                   t1 = t1,
                                   dt = dt,
                                   max_step = dt,
                                   first_step = 1e-4,
                                   atol = atol,
                                   rtol = rtol,
                                   is_disturb = is_disturb,
                                   is_dyn_ctrl = is_dyn_ctrl)

#----------------------------------------Initialization : : logger
date = datetime.now().strftime("%Y-%m-%d")
time = datetime.now().strftime("%Hh%Mm%Ss")
datafiles = [None] * Nruns

data_folder = 'simdata/' + ctrl_mode + "/Init_angle_{}_seed_{}_Nactor_{}".format(str(state_init[2]), seed, Nactor)

if is_log_data:
    pathlib.Path(data_folder).mkdir(parents=True, exist_ok=True) 

for k in range(0, Nruns):
    datafiles[k] = data_folder + '/' + my_sys.name + '_' + ctrl_mode + '_' + date + '_' + time + '__run{run:02d}.csv'.format(run=k+1)
    
    if is_log_data:
        print('Logging data to:    ' + datafiles[k])
            
        with open(datafiles[k], 'w', newline='') as outfile:
            writer = csv.writer(outfile)
            writer.writerow(['System', my_sys.name ] )
            writer.writerow(['Controller', ctrl_mode ] )
            writer.writerow(['dt', str(dt) ] )
            writer.writerow(['state_init', str(state_init) ] )
            writer.writerow(['Nactor', str(Nactor) ] )
            writer.writerow(['pred_step_size_multiplier', str(pred_step_size_multiplier) ] )
            writer.writerow(['buffer_size', str(buffer_size) ] )
            writer.writerow(['run_obj_struct', str(run_obj_struct) ] )
            writer.writerow(['R1_diag', str(R1_diag) ] )
            writer.writerow(['R2_diag', str(R2_diag) ] )
            writer.writerow(['Ncritic', str(Ncritic) ] )
            writer.writerow(['gamma', str(gamma) ] )
            writer.writerow(['critic_period_multiplier', str(critic_period_multiplier) ] )
            writer.writerow(['critic_struct', str(critic_struct) ] )
            writer.writerow(['actor_struct', str(actor_struct) ] )   
            writer.writerow(['t [s]', 'x [m]', 'y [m]', 'alpha [rad]', 'run_obj', 'accum_obj', 'v [m/s]', 'omega [rad/s]'] )

# Do not display annoying warnings when print is on
if is_print_sim_step:
    warnings.filterwarnings('ignore')
    
my_logger = loggers.Logger3WRobotNI()

#----------------------------------------Main loop
state_full_init = my_simulator.state_full

class N_CTRL:
    def pure_loop(self, observation, goal=[0, 0, 0]):
        x_robot = observation[0]
        y_robot = observation[1]
        theta = observation[2]
        x_goal = 0
        y_goal = 0
        theta_goal = 0
        
        error_x = x_goal - x_robot
        error_y = y_goal - y_robot
        error_theta = theta_goal - theta

        rho = np.sqrt(error_x**2 + error_y**2)
        alpha = -theta + np.arctan2(error_y, error_x)
        beta = error_theta - alpha
        
        # k_rho = 2
        # k_alpha = 14
        # k_beta = -1.5

        k_rho = 0.15
        k_alpha = 0.17
        k_beta = -0.05

        w = k_alpha*alpha + k_beta*beta
        v = k_rho*rho

        while alpha > np.pi:
            alpha -= 2* np.pi

        while alpha < -np.pi:
            alpha += 2* np.pi

        if -np.pi < alpha <= -np.pi / 2 or np.pi / 2 < alpha <= np.pi:
            v = -v

        return [v,w]

class ControllerOptimalPredictive:
    
    def __init__(self,
                 dim_input,
                 dim_output,
                 mode='MPC',
                 ctrl_bnds=[],
                 action_init = [],
                 t0=0,
                 sampling_time=0.1,
                 Nactor=1,
                 pred_step_size=1,
                 sys_rhs=[],
                 sys_out=[],
                 state_sys=[],
                 buffer_size=20,
                 gamma=1,
                 Ncritic=4,
                 critic_period=0.1,
                 critic_struct='quad-nomix',
                 run_obj_struct='quadratic',
                 run_obj_pars=[],
                 observation_target=[],
                 state_init=[],
                 obstacle=[],
                 seed=1):
        
        self.run_obj_struct = run_obj_struct
        self.run_obj_pars = run_obj_pars
        self.obstacle_position = None
        self.obstacle_var_x = None
        self.obstacle_var_y = None
        self.obstacle_pos = None 
        self.dim_input = dim_input
        self.dim_output = dim_output
   
        
        np.random.seed(seed)
        print(seed)

        self.dim_input = dim_input
        self.dim_output = dim_output
        
        self.mode = mode

        self.ctrl_clock = t0
        self.sampling_time = sampling_time
        
        # Controller: common
        self.Nactor = Nactor 
        self.pred_step_size = pred_step_size
        
        self.action_min = np.array( ctrl_bnds[:,0] )
        self.action_max = np.array( ctrl_bnds[:,1] )
        self.action_sqn_min = rep_mat(self.action_min, 1, Nactor)
        self.action_sqn_max = rep_mat(self.action_max, 1, Nactor) 
        self.action_sqn_init = []
        self.state_init = []

        if len(action_init) == 0:
            self.action_curr = self.action_min/10
            self.action_sqn_init = rep_mat( self.action_min/10 , 1, self.Nactor)
            self.action_init = self.action_min/10
        else:
            self.action_curr = action_init
            self.action_sqn_init = rep_mat( action_init , 1, self.Nactor)
        
        
        self.action_buffer = np.zeros( [buffer_size, dim_input] )
        self.observation_buffer = np.zeros( [buffer_size, dim_output] )        
        
        # Exogeneous model's things
        self.sys_rhs = sys_rhs
        self.sys_out = sys_out
        self.state_sys = state_sys   
        
        # Learning-related things
        self.buffer_size = buffer_size
        self.critic_clock = t0
        self.gamma = gamma
        self.Ncritic = Ncritic
        self.Ncritic = np.min([self.Ncritic, self.buffer_size-1]) # Clip critic buffer size
        self.critic_period = critic_period
        self.critic_struct = critic_struct
        self.run_obj_struct = run_obj_struct
        self.run_obj_pars = run_obj_pars
        self.observation_target = observation_target
        
        self.accum_obj_val = 0
        print('---Critic structure---', self.critic_struct)


        if self.critic_struct == 'quad-lin':
            self.dim_critic = int( ( ( self.dim_output + self.dim_input ) + 1 ) * ( self.dim_output + self.dim_input )/2 + (self.dim_output + self.dim_input) ) 
            self.Wmin = -1e3*np.ones(self.dim_critic) 
            self.Wmax = 1e3*np.ones(self.dim_critic) 
        elif self.critic_struct == 'quadratic':
            self.dim_critic = int( ( ( self.dim_output + self.dim_input ) + 1 ) * ( self.dim_output + self.dim_input )/2 )
            self.Wmin = np.zeros(self.dim_critic) 
            self.Wmax = 1e3*np.ones(self.dim_critic)    
        elif self.critic_struct == 'quad-nomix':
            self.dim_critic = self.dim_output + self.dim_input
            self.Wmin = np.zeros(self.dim_critic) 
            self.Wmax = 1e3*np.ones(self.dim_critic)    
        elif self.critic_struct == 'quad-mix':
            self.dim_critic = int( self.dim_output + self.dim_output * self.dim_input + self.dim_input )
            self.Wmin = -1e3*np.ones(self.dim_critic)  
            self.Wmax = 1e3*np.ones(self.dim_critic) 
        elif self.critic_struct == 'poly3':
            self.dim_critic = int( ( ( self.dim_output + self.dim_input ) + 1 ) * ( self.dim_output + self.dim_input ) )
            self.Wmin = -1e3*np.ones(self.dim_critic)  
            self.Wmax = 1e3*np.ones(self.dim_critic) 
        elif self.critic_struct == 'poly4':
            self.dim_critic = int( ( ( self.dim_output + self.dim_input ) + 1 ) * ( self.dim_output + self.dim_input )/2 * 3)
            self.Wmin = np.zeros(self.dim_critic) 
            self.Wmax = np.ones(self.dim_critic) 
       

    def reset(self,t0):
 
        # Controller: common

        if len(self.action_init) == 0:
            self.action_curr = self.action_min/10
            self.action_sqn_init = rep_mat( self.action_min/10 , 1, self.Nactor)
            self.action_init = self.action_min/10
        else:
            self.action_curr = self.action_init
            self.action_sqn_init = rep_mat( self.action_init , 1, self.Nactor)
        
        self.action_buffer = np.zeros( [self.buffer_size, self.dim_input] )
        self.observation_buffer = np.zeros( [self.buffer_size, self.dim_output] )        

        self.critic_clock = t0
        self.ctrl_clock = t0
    
    def receive_sys_state(self, state):
        """
        Fetch exogenous model state. Used in some controller modes. See class documentation.

        """
        self.state_sys = state
    
    def upd_accum_obj(self, observation, action):
 
        self.accum_obj_val += self.run_obj(observation, action)*self.sampling_time
    
    def initialize_obstacles(self, obstacle):
        self.obstacle_position = np.array([obstacle[:-2]])
        self.obstacle_var_x = obstacle[-2]
        self.obstacle_var_y = obstacle[-2]
        self.obstacle_pos = np.array([obstacle[ :-2]])
        return obstacle

    def multivariate(self, x, mu, sigma):
        d = len(x)
        exponent = -0.5 * np.dot(np.dot((x - mu).T, np.linalg.inv(sigma)), (x - mu))
        coefficient = 1 / ((2 * np.pi) ** (d / 2) * np.sqrt(np.linalg.det(sigma)))
        
        return coefficient * np.exp(exponent)
        
        
    
    def run_obj(self, observation, action):
  
        rho = 0
        chi = np.concatenate([observation, action])

        if self.run_obj_struct == "quadratic":
            R = self.run_obj_pars[0]
            chi = np.concatenate([observation, action])
            rho = chi.T @ R @ chi
        if self.run_obj_struct == 'biquadratic':
            rho=(chi @ self.run_obj_pars[0] @ chi.T)**2 + chi @ self.run_obj_pars[0] @ chi.T
            return rho
        
        if self.obstacle_pos is not None:
            obstacle_gain = 100
            obs_cost_x = self.multivariate(observation[0], self.obstacle_position[0][0], self.obstacle_var_x)
            obs_cost_y = self.multivariate(observation[1], self.obstacle_position[0][1], self.obstacle_var_y)
            obs_cost = obs_cost_x * obs_cost_y
            rho += obstacle_gain * obs_cost
           

        return rho

    def _actor_cost(self, action_sqn, observation):

        my_action_sqn = np.reshape(action_sqn, [self.Nactor, self.dim_input])
        
        observation_sqn = np.zeros([self.Nactor, self.dim_output])
        
        # System observation prediction
        observation_sqn[0, :] = observation
        state = self.state_sys
        for k in range(1, self.Nactor):
            state = state + self.pred_step_size * self.sys_rhs([], state, my_action_sqn[k-1, :])  # Euler scheme
            
            observation_sqn[k, :] = self.sys_out(state)
        
        J = 0         
        if self.mode=='MPC':
            for k in range(self.Nactor):
                J += self.gamma**k * self.run_obj(observation_sqn[k, :], my_action_sqn[k, :])
        
        return J
    
    def _actor_optimizer(self, observation):
  
        
        actor_opt_method = 'SLSQP'
        if actor_opt_method == 'trust-constr':
            actor_opt_options = {'maxiter': 40, 'disp': False} #'disp': True, 'verbose': 2}
        else:
            actor_opt_options = {'maxiter': 40, 'maxfev': 60, 'disp': False, 'adaptive': True, 'xatol': 1e-3, 'fatol': 1e-3}
       
        isGlobOpt = 0
        
        my_action_sqn_init = np.reshape(self.action_sqn_init, [self.Nactor*self.dim_input,])
        
        bnds = sp.optimize.Bounds(self.action_sqn_min, self.action_sqn_max, keep_feasible=True)
        
        try:
            if isGlobOpt:
                minimizer_kwargs = {'method': actor_opt_method, 'bounds': bnds, 'tol': 1e-3, 'options': actor_opt_options}
                action_sqn = basinhopping(lambda action_sqn: self._actor_cost(action_sqn, observation),
                                          my_action_sqn_init,
                                          minimizer_kwargs=minimizer_kwargs,
                                          niter = 10).x
            else:
                action_sqn = minimize(lambda action_sqn: self._actor_cost(action_sqn, observation),
                                      my_action_sqn_init,
                                      method=actor_opt_method,
                                      tol=1e-3,
                                      bounds=bnds,
                                      options=actor_opt_options).x        

        except ValueError:
            print('Actor''s optimizer failed. Returning default action')
            action_sqn = self.action_curr
        
        return action_sqn[:self.dim_input]    # Return first action
                        
    def compute_action(self, t, observation):
      
        
        time_in_sample = t - self.ctrl_clock
        
        if time_in_sample >= self.sampling_time: # New sample
            # Update controller's internal clock
            self.ctrl_clock = t
            
            if self.mode == 'MPC':  
                
                action = self._actor_optimizer(observation)

            
            self.action_curr = action
            
            return action    
    
        else:
            return self.action_curr
        
ros_preset = ROS_preset(ctrl_mode=ControllerOptimalPredictive, 
                        state_goal=[3, 3, 0.001],
                        my_sys=my_sys,
                        state_init=state_init, 
                        my_ctrl_nominal=my_ctrl_nominal, 
                        my_ctrl_benchmarking=my_ctrl_benchm,
                        my_logger=my_logger)

ros_preset.spin(is_print_sim_step=True, is_log_data=True)

print("Target Point x: {}, y: {}, theta: {}".format(self.x_traj[target_dist], self.y_traj[target_dist], self.theta_traj[target_dist]))