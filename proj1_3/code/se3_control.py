import numpy as np
from scipy.spatial.transform import Rotation

class SE3Control(object):
    """

    """
    def __init__(self, quad_params):
        """
        This is the constructor for the SE3Control object. You may instead
        initialize any parameters, control gain values, or private state here.

        For grading purposes the controller is always initialized with one input
        argument: the quadrotor's physical parameters. If you add any additional
        input arguments for testing purposes, you must provide good default
        values!

        Parameters:
            quad_params, dict with keys specified by crazyflie_params.py

        """

        # Quadrotor physical parameters.
        self.mass            = quad_params['mass'] # kg
        self.Ixx             = quad_params['Ixx']  # kg*m^2
        self.Iyy             = quad_params['Iyy']  # kg*m^2
        self.Izz             = quad_params['Izz']  # kg*m^2
        self.arm_length      = quad_params['arm_length'] # meters
        self.rotor_speed_min = quad_params['rotor_speed_min'] # rad/s
        self.rotor_speed_max = quad_params['rotor_speed_max'] # rad/s
        self.k_thrust        = quad_params['k_thrust'] # N/(rad/s)**2
        self.k_drag          = quad_params['k_drag']   # Nm/(rad/s)**2

        # You may define any additional constants you like including control gains.
        self.inertia = np.diag(np.array([self.Ixx, self.Iyy, self.Izz])) # kg*m^2
        self.g = 9.81 # m/s^2

        # STUDENT CODE HERE
        KRZ = 70
        KRXY = 700
        # KPZ = (np.sqrt(KRZ)/10)*(np.sqrt(KRZ)/10)*.85
        # KPXY = (np.sqrt(KRXY)/10)*(np.sqrt(KRXY)/10)*.85
        KPZ = 10
        KPXY = 4
        self.Kp = np.diag(np.array([KPXY, KPXY, KPZ]))
        self.Kd = 2 * np.sqrt(self.Kp)
        self.KR = np.diag(np.array([KRXY, KRXY, KRZ]))
        self.Kw = 2 * np.sqrt(self.KR)
        # STUDENT CODE HERE

    def update(self, t, state, flat_output):
        """
        This function receives the current time, true state, and desired flat
        outputs. It returns the command inputs.

        Inputs:
            t, present time in seconds
            state, a dict describing the present state with keys
                x, position, m
                v, linear velocity, m/s
                q, quaternion [i,j,k,w]
                w, angular velocity, rad/s
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s

        Outputs:
            control_input, a dict describing the present computed control inputs with keys
                cmd_motor_speeds, rad/s
                cmd_thrust, N (for debugging and laboratory; not used by simulator)
                cmd_moment, N*m (for debugging; not used by simulator)
                cmd_q, quaternion [i,j,k,w] (for laboratory; not used by simulator)
        """
        cmd_motor_speeds = np.zeros((4,))
        cmd_thrust = 0
        cmd_moment = np.zeros((3,))
        cmd_q = np.zeros((4,))

        # STUDENT CODE HERE
        # Geometric Nonlinear Controller
        # Rotation Matrix
        R = (Rotation.from_quat(state["q"])).as_matrix()

        # Equation 26
        r_dd_des = flat_output["x_ddot"]-np.matmul(self.Kd, state["v"]-flat_output["x_dot"])\
                   - np.matmul(self.Kp, state["x"]-flat_output["x"])
        r_dd_des=np.array([[r_dd_des[0]],[r_dd_des[1]],[r_dd_des[2]]])

        # Eqn 28
        #F_des = self.mass*r_dd_des+np.squeeze(np.array([[0], [0], [self.mass*self.g]]))
        F_des = self.mass * r_dd_des + (np.array([[0], [0], [self.mass * self.g]]))

        # Eqn 29
        b3 = np.matmul(R, np.array([[0], [0], [1]]))
        u1 = np.matmul(np.transpose(b3), F_des)

        # Eqn 30 - Align b3 along desired thrust
        b3_des = F_des/np.linalg.norm(F_des)

        # Eqn 31/32 - make second basis perpendicular to b1_des and yaw
        a_psi = np.array([[np.cos(flat_output["yaw"])], [np.sin(flat_output["yaw"])], [0]])
        b2_des = np.cross(b3_des, a_psi, axis=0)/np.linalg.norm(np.cross(b3_des, a_psi, axis=0))
        # Eqn 33 - make 1st basis perpendicular to b3 and b2, fill in R matrix
        b1_des = np.cross(b2_des, b3_des, axis=0)
        R_des = np.zeros((3, 3))
        for i in range(0,3):
            R_des[i,0]=b1_des[i]
            R_des[i,1]=b2_des[i]
            R_des[i,2]=b3_des[i]

        # Eqn 34 - get orientation error
        term1=np.matmul(np.transpose(R_des), R)
        term2=np.matmul(np.transpose(R), R_des)
        e_R = 0.5*(term1 - term2)
        # Vee map
        e_R = np.array([[e_R[2, 1]], [e_R[0, 2]], [e_R[1, 0]]])

        # Eqn 35 - get torque to fix rotation error
        # Set w_des to 0 for this project
        w_des = np.zeros((3, 1))
        e_w = np.array([[state["w"][0]], [state["w"][1]],[state["w"][2]]])-w_des

        u2 = np.matmul(self.inertia, (-np.matmul(self.KR, e_R)-np.matmul(self.Kw, e_w)))

        # convert from u into forces
        u = np.array([[u1[0][0]], [u2[0][0]], [u2[1][0]], [u2[2][0]]])
        gamma = self.k_drag/self.k_thrust
        coeffMatrix = np.array([[1, 1, 1, 1],
                              [0, self.arm_length, 0, -self.arm_length],
                              [-self.arm_length, 0, self.arm_length, 0],
                              [gamma, -gamma, gamma, -gamma]])
        F = np.matmul(np.linalg.inv(coeffMatrix), u)
        # Don't command negative  forces
        for i in range(0,4):
            F[i]=F[i][0]
        F[F < 0] = 0

        cmd_motor_speeds = np.squeeze(np.sqrt(F/self.k_thrust))
        cmd_moment = np.squeeze(u2)
        cmd_thrust = u1[0][0]
        cmd_q = Rotation.from_matrix(R_des).as_quat()
        control_input = {'cmd_motor_speeds':cmd_motor_speeds,
                         'cmd_thrust':cmd_thrust,
                         'cmd_moment':cmd_moment,
                         'cmd_q':cmd_q}
        return control_input
