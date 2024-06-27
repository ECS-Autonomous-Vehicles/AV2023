#!/usr/bin/env python
"""
    Modified from https://github.com/zhm-real/MotionPlanning to make it compatible with ros 
    Author : 
        Atras Yonda Farizi
        Engineering Physics
        Institut Teknologi Sepuluh Nopember
"""
import rospy
import math
import cvxpy
import numpy as np
import tf

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, Pose
from teb_local_planner.msg import FeedbackMsg
from tf.transformations import euler_from_quaternion

class config:
    # System config
    NX = 4  # state vector: z = [x, y, v, phi]
    NU = 2  # input vector: u = [acceleration, steer]
    T = 6  # finite time horizon length

    # MPC config
    Q = np.diag([1.0, 1.0, 1.0, 1.0])  # penalty for states
    Qf = np.diag([1.0, 1.0, 1.0, 1.0])  # penalty for end state
    R = np.diag([0.01, 0.1])  # penalty for inputs
    Rd = np.diag([0.01, 0.1])  # penalty for change of inputs

    iter_max = 5  # max iteration
    target_speed = 10.0 / 3.6  # target speed

    dt = 0.1  # time step --> frequency controller = 10 Hz

    d_dist = 1.0  # dist step
    du_res = 0.1  # threshold for stopping iteration

    # vehicle config
    RF = 0.5 - 0.05  # [m] distance from rear to vehicle front end of vehicle
    RB = 0.05        # [m] distance from rear to vehicle back end of vehicle
    W = 0.3          # [m] width of vehicle
    WD = 0.3 + 2 * 0.05  # [m] distance between left-right wheels, calculated as base_width + 2 * axle_offset
    WB = 0.5 - 2 * 0.05  # [m] Wheel base, calculated as base_length - 2 * axle_offset
    TR = 0.1         # [m] Tyre radius
    TW = 0.08        # [m] Tyre width

    steer_max = np.deg2rad(45.0)  # max steering angle [rad]
    steer_change_max = np.deg2rad(30.0)  # maximum steering speed [rad/s]
    speed_max = 2  # maximum speed [m/s]
    speed_min = 0 # minimum speed [m/s]
    acceleration_max = 0.5  # maximum acceleration [m/s2]

class MPC_Node:
    def __init__(self):
        rospy.Subscriber("/odom2", Odometry, self.odom_callback, None, 100)
        rospy.Subscriber("/move_base/TebLocalPlannerROS/teb_feedback", FeedbackMsg, self.path_callback) 
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goalCB)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amclCB)
        # rospy.Subscriber("/move_base/TebLocalPlannerROS/local_plan", Path, self.path_callback)

        self.latest_odom = None
        self.latest_plan = None
        self.tf_listener = tf.TransformListener()
        self._goal_received = False
        self._goal_reached = False
        self._goal_pos = {'x': 0.0, 'y': 0.0}
        self._goalRadius = 0.05  # Set this to the appropriate radius value
        self.loop = 0
        self.v_exc = 0
        self.delta_exc = 0

    def odom_callback(self, data):
        self.latest_odom = data
        self.odom_x = self.latest_odom.pose.pose.position.x
        self.odom_y = self.latest_odom.pose.pose.position.y

        # Extract orientation quaternion
        orientation_q = self.latest_odom.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        # Convert quaternion to euler angles
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.odom_psi = yaw # in radians

        self.odom_x_dot = self.latest_odom.twist.twist.linear.x
        self.odom_y_dot = self.latest_odom.twist.twist.linear.y
        self.odom_psi_dot = self.latest_odom.twist.twist.angular.z

        return self.odom_x, self.odom_y, self.odom_psi, self.odom_x_dot, self.odom_y_dot, self.odom_psi_dot, self.latest_odom

    def path_callback(self, message):
        self.latest_plan = message.trajectories[message.selected_trajectory_idx].trajectory
        self.path_x = []
        self.path_y = []
        self.path_psi = []
        self.path_x_dot = []
        self.path_psi_dot = []
        try:
            self.tf_listener.waitForTransform('/odom', message.header.frame_id, rospy.Time(0), rospy.Duration(4.0))

            for data in self.latest_plan:
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = message.header.frame_id
                pose_stamped.header.stamp = rospy.Time(0)  # Use latest available transform
                pose_stamped.pose.position = data.pose.position
                pose_stamped.pose.orientation = data.pose.orientation
            
                transformed_pose = self.tf_listener.transformPose('/odom', pose_stamped)
                self.path_x.append(transformed_pose.pose.position.x)
                self.path_y.append(transformed_pose.pose.position.y)
                # Extract yaw from quaternion
                orientation_q = transformed_pose.pose.orientation
                orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
                self.path_psi.append(yaw)

                self.path_x_dot.append(data.velocity.linear.x)
                self.path_psi_dot.append(data.velocity.angular.z)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Transform failed: %s", str(e))
        return self.path_x, self.path_y, self.path_psi, self.path_x_dot, self.path_psi_dot, self.latest_plan
    
    def goalCB(self, goalMsg):
        self._goal_pos['x'] = goalMsg.pose.position.x
        self._goal_pos['y'] = goalMsg.pose.position.y
        self._goal_received = True
        self._goal_reached = False
        rospy.loginfo("Goal Received :goalCB!")
        return self._goal_received, self._goal_reached

    def amclCB(self, amclMsg):
        if self._goal_received:
            car2goal_x = self._goal_pos['x'] - amclMsg.pose.pose.position.x
            car2goal_y = self._goal_pos['y'] - amclMsg.pose.pose.position.y
            dist2goal = math.sqrt(car2goal_x**2 + car2goal_y**2)
            if dist2goal < self._goalRadius:
                self._goal_received = False
                self._goal_reached = True
                rospy.loginfo("Goal Reached!")
        return self._goal_received, self._goal_reached

    # def collect_car_state(self):
    #     if self.latest_plan is not None and self.latest_odom is not None:
    #         _X_ref = self.path_x
    #         _Y_ref = self.path_y
    #         _Psi_ref = self.path_psi
    #         _X_ref_dot = self.path_x_dot
    #         _Psi_ref_dot = self.path_psi_dot

    #         _X_pos = self.odom_x
    #         _Y_pos = self.odom_y
    #         _Psi_pos = self.odom_psi
    #         X_dot = self.odom_x_dot
    #         Y_dot = self.odom_y_dot
    #         Psi_dot = self.odom_psi_dot
    #     else:
    #         rospy.logwarn("Either Path/Odom/Cmd_vel Message is Missing, check /move_base local plan,/odom and /move_base/cmd_vel")
    #         _X_ref = [0, 0]
    #         _Y_ref = [0, 0]
    #         _Psi_ref = [0, 0]
    #         _X_ref_dot = [0, 0]
    #         _Psi_ref_dot = [0, 0]
    #         _X_pos = 0
    #         _Y_pos = 0
    #         _Psi_pos = 0
    #         X_dot = 0
    #         Y_dot = 0
    #         Psi_dot = 0
    #     return _X_ref, _Y_ref, _Psi_ref, _X_pos, _Y_pos, _Psi_pos, X_dot, Y_dot,Psi_dot,_X_ref_dot,_Psi_ref_dot
    
    def run(self):
        if not self._goal_received and not self._goal_reached :
            rospy.loginfo("Waiting for Goal")
        
        if not self._goal_received and self._goal_reached:
            rospy.loginfo("Goal is Reached")
            self.loop = 0
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = 0
            cmd_vel_msg.angular.z = 0
            pub.publish(cmd_vel_msg)
            rospy.loginfo("Published x_dot: %f and psi_dot: %f to /cmd_vel", cmd_vel_msg.linear.x, cmd_vel_msg.angular.z)

        if self._goal_received and not self._goal_reached:
            rospy.loginfo("Starting Control Loop")
            if self.latest_plan is None or self.latest_odom is None:
                self.skip_loop()
                print("SKIP CASE 1")
            elif len(self.path_x)<3 or len(self.path_y)<3 or len(self.path_psi)<3 or len(self.path_x_dot)<3 :
                print("SKIP CASE 2")
                self.skip_loop()
            else:
                self.run_loop()
                self.publish_state()

    def publish_state(self):
        global X_ref,Y_ref,Psi_ref,X_ref_dot

        ref_pose_msg = Pose()
        odom_pose_msg = Pose()
        ref_twist_msg = Twist()
        odom_twist_msg = Twist()
        print("Loop = ", self.loop)
        # ==== CASE YG DIKIRIM REF_K-1 DAN ODOM_K ==============
        # if self.loop == 0 :
        #     X_ref = self.path_x[0]
        #     Y_ref = self.path_y[0]
        #     Psi_ref = self.path_psi[0]
        #     X_ref_dot = self.path_x_dot[0]
        # else :
        #     ref_pose_msg.position.x = X_ref
        #     ref_pose_msg.position.y = Y_ref
        #     ref_pose_msg.orientation.z = Psi_ref
        #     ref_twist_msg.linear.x = X_ref_dot

        #     odom_pose_msg.position.x = self.odom_x
        #     odom_pose_msg.position.y = self.odom_y
        #     odom_pose_msg.orientation.z = self.odom_psi
        #     odom_twist_msg.linear.x = self.odom_x_dot

        #     print("ref pose x:", ref_pose_msg.position.x)
        #     print("ref pose y:", ref_pose_msg.position.y)
        #     print("ref pose theta:", ref_pose_msg.orientation.z)
        #     print("ref velocity:", ref_twist_msg.linear.x)

        #     print("odom pose x:", odom_pose_msg.position.x)
        #     print("odom pose y:", odom_pose_msg.position.y)
        #     print("odom pose theta:", odom_pose_msg.orientation.z)
        #     print("odom velocity:", odom_twist_msg.linear.x)

        #     # Publish message
        #     ref_pose_pub.publish(ref_pose_msg)
        #     ref_twist_pub.publish(ref_twist_msg)
        #     odom_pose_pub.publish(odom_pose_msg)
        #     odom_twist_pub.publish(odom_twist_msg)

        #     X_ref = self.path_x[0]
        #     Y_ref = self.path_y[0]
        #     Psi_ref = self.path_psi[0]
        #     X_ref_dot = self.path_x_dot[0]
        # self.loop = self.loop + 1

        # ==== CASE YG DIKIRIM REF_K DAN ODOM_K ==============
        ref_pose_msg.position.x = self.path_x[0]
        ref_pose_msg.position.y = self.path_y[0]
        ref_pose_msg.orientation.z = self.path_psi[0]
        ref_twist_msg.linear.x = self.path_x_dot[0]

        odom_pose_msg.position.x = self.odom_x
        odom_pose_msg.position.y = self.odom_y
        odom_pose_msg.orientation.z = self.odom_psi
        odom_twist_msg.linear.x = self.odom_x_dot

        print("ref pose x:", ref_pose_msg.position.x)
        print("ref pose y:", ref_pose_msg.position.y)
        print("ref pose theta:", ref_pose_msg.orientation.z)
        print("ref velocity:", ref_twist_msg.linear.x)

        print("odom pose x:", odom_pose_msg.position.x)
        print("odom pose y:", odom_pose_msg.position.y)
        print("odom pose theta:", odom_pose_msg.orientation.z)
        print("odom velocity:", odom_twist_msg.linear.x)

        # Publish message
        ref_pose_pub.publish(ref_pose_msg)
        ref_twist_pub.publish(ref_twist_msg)
        odom_pose_pub.publish(odom_pose_msg)
        odom_twist_pub.publish(odom_twist_msg)


    def run_loop(self):
        _X_ref = self.path_x
        _Y_ref = self.path_y
        _Psi_ref = self.path_psi
        _X_ref_dot = self.path_x_dot

        _X_pos = self.odom_x
        _Y_pos = self.odom_y
        _Psi_pos = self.odom_psi
        X_dot = self.odom_x_dot

        if (len(_X_ref)>config.T):
            N = config.T
        else:
            N = len(_X_ref)-1

        # print("ref_x ", len(_X_ref))
        # print("ref_y ", len(_Y_ref))
        # print("ref_psi ", len(_Psi_ref))
        # print("speed_profile", len(_X_ref_dot))
        delta_opt, a_opt = None, None
        a_exc, delta_exc = 0.0, 0.0

        z_ref = np.zeros((config.NX, N + 1))
        z_ref[0] = _X_ref[0:N+ 1]
        z_ref[1] = _Y_ref[0:N+ 1]
        z_ref[2] = _X_ref_dot[0:N+ 1]
        z_ref[3] = _Psi_ref[0:N+ 1]

        z0 = [_X_pos, _Y_pos, X_dot, _Psi_pos]

        a_opt, delta_opt, x_opt, y_opt, yaw_opt, v_opt = linear_mpc_control(z_ref, z0, a_opt, delta_opt, N)

        if delta_opt is not None:
            self.delta_exc = delta_opt[0]
            self.v_exc = v_opt[1]
        # print("=============")
        # print("x_opt", x_opt)
        # print("y_opt", y_opt)
        # print("yaw_opt", yaw_opt)
        print("v_opt", v_opt)
        print("a_opt", a_opt)
        # print("delta_exc", delta_exc)
        # print("a_exc", a_exc)

        cmd_vel_msg = Twist()
        # cmd_vel_msg.linear.x = a_exc
        cmd_vel_msg.linear.x = v_opt[1]
        cmd_vel_msg.angular.z = delta_exc
        pub.publish(cmd_vel_msg)
        rospy.loginfo("Published x_dot: %f and psi_dot: %f to /cmd_vel", cmd_vel_msg.linear.x, cmd_vel_msg.angular.z)

    def skip_loop(self):
        print("========== Skip the loop =============")
        cmd_vel_msg = Twist()
        # cmd_vel_msg.linear.x = a_exc
        cmd_vel_msg.linear.x = self.v_exc
        cmd_vel_msg.angular.z = self.delta_exc
        pub.publish(cmd_vel_msg)
        rospy.loginfo("Published x_dot: %f and psi_dot: %f to /cmd_vel", cmd_vel_msg.linear.x, cmd_vel_msg.angular.z)

    


class Car_States:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, direct=1.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.direct = direct

    def update(self, a, delta, direct):
        delta = self.limit_input_delta(delta)
        self.x += self.v * math.cos(self.yaw) * config.dt
        self.y += self.v * math.sin(self.yaw) * config.dt
        self.yaw += self.v / config.WB * math.tan(delta) * config.dt
        self.direct = direct
        self.v += self.direct * a * config.dt
        self.v = self.limit_speed(self.v)

    @staticmethod
    def limit_input_delta(delta):
        if delta >= config.steer_max:
            return config.steer_max

        if delta <= -config.steer_max:
            return -config.steer_max

        return delta

    @staticmethod
    def limit_speed(v):
        if v >= config.speed_max:
            return config.speed_max

        if v <= config.speed_min:
            return config.speed_min

        return v

def pi_2_pi(angle):
    if angle > math.pi:
        return angle - 2.0 * math.pi

    if angle < -math.pi:
        return angle + 2.0 * math.pi

    return angle

def calc_ref_trajectory_in_T_step(node, ref_path, sp):
    """
    calc referent trajectory in T steps: [x, y, v, yaw]
    using the current velocity, calc the T points along the reference path
    :param node: current information
    :param ref_path: reference path: [x, y, yaw]
    :param sp: speed profile (designed speed strategy)
    :return: reference trajectory
    """

    z_ref = np.zeros((config.NX, config.T + 1))
    length = ref_path.length

    ind, _ = ref_path.nearest_index(node)

    z_ref[0, 0] = ref_path.cx[ind]
    z_ref[1, 0] = ref_path.cy[ind]
    z_ref[2, 0] = sp[ind]
    z_ref[3, 0] = ref_path.cyaw[ind]

    dist_move = 0.0

    for i in range(1, config.T + 1):
        dist_move += abs(node.v) * config.dt
        ind_move = int(round(dist_move / config.d_dist))
        index = min(ind + ind_move, length - 1)

        z_ref[0, i] = ref_path.cx[index]
        z_ref[1, i] = ref_path.cy[index]
        z_ref[2, i] = sp[index]
        z_ref[3, i] = ref_path.cyaw[index]

    return z_ref, ind

def linear_mpc_control(z_ref, z0, a_old, delta_old, N):
    """
    linear mpc controller
    :param z_ref: reference trajectory in T steps
    :param z0: initial state vector
    :param a_old: acceleration of T steps of last time
    :param delta_old: delta of T steps of last time
    :return: acceleration and delta strategy based on current information
    """

    if a_old is None or delta_old is None:
        a_old = [0.0] * N
        delta_old = [0.0] * N

    x, y, yaw, v = None, None, None, None

    for k in range(config.iter_max):
        z_bar = predict_states_in_T_step(z0, a_old, delta_old, z_ref, N)
        a_rec, delta_rec = a_old[:], delta_old[:]
        a_old, delta_old, x, y, yaw, v = solve_linear_mpc(z_ref, z_bar, z0, delta_old, N)

        # du_a_max = max([abs(ia - iao) for ia, iao in zip(a_old, a_rec)])
        # du_d_max = max([abs(ide - ido) for ide, ido in zip(delta_old, delta_rec)])

        # if max(du_a_max, du_d_max) < config.du_res:
        #     break

    return a_old, delta_old, x, y, yaw, v


def predict_states_in_T_step(z0, a, delta, z_ref, N):
    """
    given the current state, using the acceleration and delta strategy of last time,
    predict the states of vehicle in T steps.
    :param z0: initial state
    :param a: acceleration strategy of last time
    :param delta: delta strategy of last time
    :param z_ref: reference trajectory
    :return: predict states in T steps (z_bar, used for calc linear motion model)
    """

    z_bar = z_ref * 0.0

    for i in range(config.NX):
        z_bar[i, 0] = z0[i]

    node = Car_States(x=z0[0], y=z0[1], v=z0[2], yaw=z0[3])

    for ai, di, i in zip(a, delta, range(1, N + 1)):
        node.update(ai, di, 1.0)
        z_bar[0, i] = node.x
        z_bar[1, i] = node.y
        z_bar[2, i] = node.v
        z_bar[3, i] = node.yaw

    return z_bar

def calc_linear_discrete_model(v, phi, delta):
    """
    calc linear and discrete time dynamic model.
    :param v: speed: v_bar
    :param phi: angle of vehicle: phi_bar
    :param delta: steering angle: delta_bar
    :return: A, B, C
    """

    A = np.array([[1.0, 0.0, config.dt * math.cos(phi), - config.dt * v * math.sin(phi)],
                  [0.0, 1.0, config.dt * math.sin(phi), config.dt * v * math.cos(phi)],
                  [0.0, 0.0, 1.0, 0.0],
                  [0.0, 0.0, config.dt * math.tan(delta) / config.WB, 1.0]])

    B = np.array([[0.0, 0.0],
                  [0.0, 0.0],
                  [config.dt, 0.0],
                  [0.0, config.dt * v / (config.WB * math.cos(delta) ** 2)]])

    C = np.array([config.dt * v * math.sin(phi) * phi,
                  -config.dt * v * math.cos(phi) * phi,
                  0.0,
                  -config.dt * v * delta / (config.WB * math.cos(delta) ** 2)])

    return A, B, C

def solve_linear_mpc(z_ref, z_bar, z0, d_bar, N):
    """
    solve the quadratic optimization problem using cvxpy, solver: OSQP
    :param z_ref: reference trajectory (desired trajectory: [x, y, v, yaw])
    :param z_bar: predicted states in T steps
    :param z0: initial state
    :param d_bar: delta_bar
    :return: optimal acceleration and steering strategy
    """

    z = cvxpy.Variable((config.NX, N + 1))
    u = cvxpy.Variable((config.NU, N))

    cost = 0.0
    constrains = []

    for i in range(N):
        cost += cvxpy.quad_form(u[:, i], config.R)
        cost += cvxpy.quad_form(z_ref[:, i] - z[:, i], config.Q)

        A, B, C = calc_linear_discrete_model(z_bar[2, i], z_bar[3, i], d_bar[i])

        constrains += [z[:, i + 1] == A @ z[:, i] + B @ u[:, i] + C]

        if i < N - 1:
            cost += cvxpy.quad_form(u[:, i + 1] - u[:, i], config.Rd)
            constrains += [cvxpy.abs(u[1, i + 1] - u[1, i]) <= config.steer_change_max * config.dt]

    cost += cvxpy.quad_form(z_ref[:, N] - z[:, N], config.Qf)

    constrains += [z[:, 0] == z0]
    constrains += [z[2, :] <= config.speed_max]
    constrains += [z[2, :] >= config.speed_min]
    constrains += [cvxpy.abs(u[0, :]) <= config.acceleration_max]
    constrains += [cvxpy.abs(u[1, :]) <= config.steer_max]

    prob = cvxpy.Problem(cvxpy.Minimize(cost), constrains)
    prob.solve(solver=cvxpy.OSQP)

    a, delta, x, y, yaw, v = None, None, None, None, None, None

    if prob.status == cvxpy.OPTIMAL or \
            prob.status == cvxpy.OPTIMAL_INACCURATE:
        x = z.value[0, :]
        y = z.value[1, :]
        v = z.value[2, :]
        yaw = z.value[3, :]
        a = u.value[0, :]
        delta = u.value[1, :]
    else:
        print("Cannot solve linear mpc!")
        x = [0.0]*(N + 1)
        y = [0.0]*(N + 1)
        v = [0.0]*(N + 1)
        yaw = [0.0]*(N + 1)
        a = [0.0] * N
        delta = [0.0] * N
    return a, delta, x, y, yaw, v

if __name__ == '__main__':
    rospy.init_node('controller_mpc_node')
    rospy.loginfo("Node has been started")
    rate = rospy.Rate(10)
    mpc_node = MPC_Node()
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10 )
    ref_pose_pub = rospy.Publisher("/state/ref_pose", Pose, queue_size=10 )
    odom_pose_pub = rospy.Publisher("/state/odom_pose", Pose, queue_size=10 )
    ref_twist_pub = rospy.Publisher("/state/ref_twist", Twist, queue_size=10 )
    odom_twist_pub = rospy.Publisher("/state/odom_twist", Twist, queue_size=10 )
    while not rospy.is_shutdown():
        mpc_node.run()
        rate.sleep()
