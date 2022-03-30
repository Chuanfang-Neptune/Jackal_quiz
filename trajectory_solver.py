import yaml
import os
import argparse

import numpy as np
import casadi as ca
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
class Jackal_quiz:
    def __init__(self):
        # pose info
        self.robot_pose_x = 0 #meter
        self.robot_pose_y = 0 #meter
        self.robot_pose_z = 0 #radian
        # target
        self.target_pose_x = 3 #meter
        self.target_pose_y = 6 #meter
        self.target_pose_z = 0 #radian
        # velocity
        self.vx = 0
        self.vw = 0
        # dynamics limit
        self.vx_limit = 0.6 #m/s
        self.vw_limit = 1.2 #rad/s
        self.ax_limit = 10 #m/s^2
        self.aw_limit = 12 #rad/s^2
        # planned trajectory
        self.step = int(np.hypot(self.target_pose_x,self.target_pose_y))*40
        self.planned_traj = np.empty((self.step,3)) #trajectory
        self.det_t = 1000
        # pid parameters
        # current state error
        self.z_err = 0.
        self.pos_err = 0.
        # initial state error
        self.int_z = 0.
        self.int_pos = 0.
        # previous state error
        self.prev_z_err = 0.
        self.prev_pos_err = 0.
        # default pd parameters
        self.w_pid = [20, 20, 0.5]
        self.v_pid = [4, 2, 0]
        # ros
        rospy.init_node("jackal_quiz", anonymous = True, disable_signals=True)
        rospy.loginfo('Node "jackal_quiz" created')
        # cmd output
        self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    
    def odometryCb(self,Odometry):
        """callback function for jackal localization with filtered odometry """
        orientation_q = Odometry.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.robot_pose_z) = euler_from_quaternion (orientation_list)
        self.robot_pose_x = Odometry.pose.pose.position.x
        self.robot_pose_y = Odometry.pose.pose.position.y

    def gazebo_model_Cb(self,ModelStates):
        """callback function for jackal localization with ground truth from Gazebo (assume a camera looking down on robot)"""
        orientation_q = ModelStates.pose[1].orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.robot_pose_z) = euler_from_quaternion (orientation_list)
        self.robot_pose_x = ModelStates.pose[1].position.x
        self.robot_pose_y = ModelStates.pose[1].position.y

    @staticmethod
    def l2_tar_distance(px,py,tx,ty):
        """calculate the l2 distance error from target to current position"""
        return np.hypot(tx-px,ty-py)
    @staticmethod
    def reg_ang(angle):
        """regulate the angle between -pi to pi"""
        if angle > np.pi:
            angle -= 2*np.pi
        elif angle < -np.pi:
            angle += 2*np.pi
        return angle
    @staticmethod
    def pid_control(err, prev_err, integrator, pid, u_max):
        """pid control and integrator anti-windup"""
        Ts = 1/10  # 10Hz
        integrator = integrator + Ts*(err+prev_err)/2
        derivative = (err-prev_err)/Ts
        u = pid[0]*err + pid[1]*integrator + pid[2]*derivative
        if u > u_max:
            if pid[1] != 0:
                integrator = integrator - Ts*(err+prev_err)/2
            u = u_max
        elif u < -u_max:
            if pid[1] != 0:
                integrator = integrator - Ts*(err+prev_err)/2
            u = -u_max
        return integrator, u

    def traj_following(self):
        """follow planned trajectory with PID control at fixed frequency"""
        # Twist template
        velocity = Twist()
        velocity.linear.x = 0
        velocity.linear.y = 0
        velocity.linear.z = 0
        velocity.angular.x = 0
        velocity.angular.y = 0
        velocity.angular.z = 0
        # loop trajectory following at 50Hz
        loop_rate = rospy.Rate(50)
        # initial target
        next_point, self.planned_traj = self.planned_traj[0,:], self.planned_traj[1:,:]
        while not rospy.is_shutdown():
            self.pos_err = self.l2_tar_distance(self.robot_pose_x,self.robot_pose_y,next_point[0],next_point[1])
            self.z_err = self.reg_ang(next_point[2]-self.robot_pose_z)
            if (self.planned_traj.shape[0]==0):
                #stop if last point reached
                velocity.linear.x = 0
                velocity.angular.z = 0
                self.pub_cmd.publish(velocity)
                break
            # rospy.loginfo('cx{} cy{} tx{} ty{} err{}'.format(self.robot_pose_x,self.robot_pose_y,next_point[0],next_point[1],self.pos_err))
            if (self.planned_traj.shape[0] > int(self.step*0.1)):
                if (self.z_err < 0.1 and self.pos_err < 0.15):
                    next_point, self.planned_traj = self.planned_traj[0,:], self.planned_traj[1:,:]
                    rospy.loginfo('next point: {},{},{}'.format(next_point[0],next_point[1],next_point[2]))
            else:
                if (self.pos_err < 0.17):
                    next_point, self.planned_traj = self.planned_traj[0,:], self.planned_traj[1:,:]
                    rospy.loginfo('next point: {},{},{}'.format(next_point[0],next_point[1],next_point[2]))
            #pid control and state variables update
            self.int_z, velocity.angular.z = self.pid_control(self.z_err, self.prev_z_err, self.int_z, self.w_pid, self.vw_limit)
            self.int_pos, velocity.linear.x = self.pid_control(self.pos_err, self.prev_pos_err, self.int_pos, self.v_pid, self.vx_limit)
            self.pub_cmd.publish(velocity)
            loop_rate.sleep()

    def open_loop_traj_following(self):
        """
        open-loop trajectory following assuming perfect motor acceleration and odometry
        publish velocity repeatedly every det_t slice that is closest to the planned optimal control outputs
        """
        velocity = Twist()
        velocity.linear.x = 0
        velocity.linear.y = 0
        velocity.linear.z = 0
        velocity.angular.x = 0
        velocity.angular.y = 0
        velocity.angular.z = 0
        # loop trajectory following
        loop_rate = rospy.Rate(300)
        i = 0
        start_time = rospy.Time().now().to_sec()
        while not rospy.is_shutdown():
            velocity.linear.x = self.v_list[i][0]
            velocity.angular.z = self.v_list[i][1]
            self.pub_cmd.publish(velocity)
            loop_rate.sleep()
            if (rospy.Time().now().to_sec() - start_time - i*self.det_t) >= -0.005 :
                print(i,velocity.linear.x,velocity.linear.z,self.planned_traj[i,0],self.planned_traj[i,1],self.robot_pose_x,self.robot_pose_y)
                i = i+1
                if i>=jq.step:
                    break

from tf.transformations import euler_from_quaternion
def quat2euler(msg):
    """math utils for converting quat to euler"""
    orientation_q = msg.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    return yaw

def solveProblem(tar_x,tar_y,tar_z,ax_limit,aw_limit,vx_limit,vw_limit,step):
    """
    custom MPC trajectory planner optimizing the time to reach the target
    given the velocity and acceleration boundary and target pose
    The trajectory is planned under ideal assumption that the motor can indeed
    reproduce the planned acceleration and velocity.
    """
    print_time = rospy.Time().now().to_sec() #perf_counter
    N = step*2+1 #step
    # control input [ax0,aw0,ax1,aw1,...,T]
    U = ca.SX.sym('U', N)
    # start state still
    x0 = ca.SX.sym('x0', 5)    
    x0[0] = 0
    x0[1] = 0
    x0[2] = 0
    x0[3] = 0
    x0[4] = 0
    # end state initial value
    xf = ca.SX.sym('xf', 5)    
    xf[0] = x0[0]
    xf[1] = x0[1]
    xf[2] = x0[2]
    xf[3] = x0[3]
    xf[4] = x0[4]
    # objective
    obj = U[-1] # optimize total time
    # constraints
    g = []
    # time step
    det_t = ca.SX.sym('det_t', 1)
    det_t = U[-1] / ((N - 1) / 2)
    # iteratively update constraints and states equations
    for i in range (int((N - 1) / 2)):
        xf[4] += det_t * U[2 * i + 1]
        xf[3] += det_t * U[2 * i]
        xf[2] += det_t * xf[4]
        xf[1] += det_t * xf[3] * ca.sin(xf[2])
        xf[0] += det_t * xf[3] * ca.cos(xf[2])
        g.append(xf[3])
        g.append(xf[4])
    g.append(xf[0])
    g.append(xf[1])
    g.append(xf[2])
    g.append(xf[3])
    g.append(xf[4])
    # solver casadi def
    nlp_prob = {'f': obj, 'x': U, 'g': ca.vertcat(*g)}
    opts_setting = {'ipopt.max_iter':100, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-8, 'ipopt.acceptable_obj_change_tol':1e-6}
    solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)
    lbx = []
    ubx = []
    lbg = []
    ubg = []
    # apply boundary (respect velocity/acceleration boundary)
    for i in range (int((N - 1) / 2)):
        lbx.append(-ax_limit)
        ubx.append(ax_limit)
        lbx.append(-aw_limit)
        ubx.append(aw_limit)
        lbg.append(-vx_limit)
        ubg.append(vx_limit)
        lbg.append(-vw_limit)
        ubg.append(vw_limit)
    lbx.append(0)
    ubx.append(100)
    # target constraints (still at target pose)
    lbg.append(tar_x)    
    ubg.append(tar_x)    
    lbg.append(tar_y)    
    ubg.append(tar_y)    
    lbg.append(tar_z)    
    ubg.append(tar_z)    
    lbg.append(0)    
    ubg.append(0)    
    lbg.append(0)    
    ubg.append(0)   
    # solve problem
    res = solver(lbx = lbx, ubx = ubx, lbg = lbg, ubg = ubg)
    print('cost time of casadi opt problem: ', rospy.Time().now().to_sec() - print_time)
    #print(res['f'], res['x'])
    UU = res['x']

    # results
    xf[0] = x0[0]
    xf[1] = x0[1]
    xf[2] = x0[2]
    xf[3] = x0[3]
    xf[4] = x0[4]
    det_t = ca.SX.sym('det_t', 1)
    det_t = UU[-1] / ((N - 1) / 2)
    
    trajectory = []
    v_list = []
    for i in range (int((N - 1) / 2)):
        xf[4] += det_t * UU[2 * i + 1]
        xf[3] += det_t * UU[2 * i]
        xf[2] += det_t * xf[4]
        xf[1] += det_t * xf[3] * ca.sin(xf[2])
        xf[0] += det_t * xf[3] * ca.cos(xf[2])
        trajectory.append([float(xf[0]),float(xf[1]),float(xf[2])])
        v_list.append([float(xf[3]),float(xf[4])])


    print('Total time: ', float(UU[-1]))
    print('Delta T: ', float(det_t))
    return (det_t,np.array(trajectory),np.array(v_list))


class CustomFormatter(argparse.ArgumentDefaultsHelpFormatter,
                      argparse.RawDescriptionHelpFormatter):
    pass


def cli(): 
    """parser for incoming arguments""" 
    parser = argparse.ArgumentParser(
        prog='python3 trajectory_solver',
        usage='%(prog)s [options]',
        description=__doc__,
        formatter_class=CustomFormatter,
    )
    parser.add_argument('--yaml', default=None, nargs='?', const=True,
                        help='target.yml file name')
    parser.add_argument('--open_loop', default=False, nargs='?', const=True,
                        help='open_loop mode')
    parser.add_argument('--gazebo_camera', default=False, nargs='?', const=True,
                        help="get robot's position from gazebo ground truth")
    args = parser.parse_args()
    return args


if __name__ == '__main__':
    jq = Jackal_quiz()
    args = cli()
    if args.yaml:
        curPath = os.path.dirname(os.path.realpath(__file__))
        yamlPath = os.path.join(curPath, args.yaml)
        f = open(yamlPath, 'r', encoding='utf-8')
        cfg = f.read()
        d = yaml.safe_load(cfg)
        #display loaded param set
        print(d)
        #update params with yml config
        jq.target_pose_x = d["tar"]["target_pose_x"]
        jq.target_pose_y = d["tar"]["target_pose_y"]
        jq.target_pose_z = d["tar"]["target_pose_z"]
        jq.ax_limit = d["limit"]["ax_limit"]
        jq.vx_limit = d["limit"]["vx_limit"]
        jq.vw_limit = d["limit"]["vw_limit"]
        jq.aw_limit = d["limit"]["aw_limit"]
        jq.w_pid = d["pid"]["w_pid"]
        jq.v_pid = d["pid"]["v_pid"]
    if args.gazebo_camera:
        #subscribe to ground truth
        jq.robot_pose_sub = rospy.Subscriber('/gazebo/model_states',ModelStates,jq.gazebo_model_Cb)
    else:
        #subscribe to sensor filtered output
        jq.robot_pose_sub = rospy.Subscriber('/odometry/filtered',Odometry,jq.odometryCb)

    jq.det_t,jq.planned_traj,jq.v_list = solveProblem(jq.target_pose_x,jq.target_pose_y,\
                                   jq.target_pose_z,jq.ax_limit,\
                                   jq.aw_limit,jq.vx_limit,jq.vw_limit,jq.step)
    #pause program and show planned trajectory
    plt.plot(jq.planned_traj[:,0],jq.planned_traj[:,1])
    plt.show()
    if args.open_loop:
        jq.open_loop_traj_following()
    else:
        jq.traj_following()
    print('target reached')