import numpy as np
import cvxopt
from scipy.special import factorial
from proj1_3.code.graph_search import graph_search


class WorldTraj(object):
    """
    Given a world with a start and a goal, use graph_search.py to generate a path, then generate a polynomial trajectory
    using a sparsified version of that path. Update function provides the desired state at the provided time. Other
    functions are internal for reducing dense path or setting up trajectory constraints.
    """

    def __init__(self, world, start, goal):
        """
        This is the constructor for the trajectory object. A fresh trajectory
        object will be constructed before each mission. For a world trajectory,
        the input arguments are start and end positions and a world object.

        Computes polynomial coefficients for desired path when initiated.

        Parameters:
            world, World object representing the environment obstacles
            start, xyz position in meters, shape=(3,)
            goal,  xyz position in meters, shape=(3,)

        """
        # These parameters can be tweaked depending on the map
        self.resolution = np.array([0.25, 0.25, 0.25])
        self.margin = 0.5
        # margin for point elimination in douglas peucker
        self.eps = self.margin / .5

        # Dense path returned from Dijkstra (astar = false) or AStar
        self.path = graph_search(world, self.resolution, self.margin, start, goal, astar=True)
        self.goal = goal
        # Sparse waypoints, reduced via douglas peucker if path exists, returned empty if there is none
        if self.path is None:
            self.points = None
            self.xcoeff = None
        else:
            self.points = np.concatenate((dougPeuck(self.path, self.eps), [self.path[-1, :]]), axis=0)

            # Calculating trajectory through sparse waypoints
            self.travelvel = 2.65  # approximate, just to get times for each segment
            self.numWaypoints = self.points.shape[0]
            self.oldTime = 0
            n = self.numWaypoints - 1  # number of polynomial segments
            d = 8  # the terms in each polynomial, 8 for 7th order min snap
            dist_between_waypoints = np.linalg.norm(np.diff(self.points, axis=0), axis=1)
            # approximate time deltas based on average speed
            dt = dist_between_waypoints / self.travelvel + .00001

            # Variable notation differs slightly from https://cvxopt.org/userguide/coneprog.html#quadratic-programming
            # H is P, Aeq is A, beq is b, but the rest are the same
            Aeq = np.zeros((0, 4 * d * n))
            beq = np.zeros((0, 1))
            G = np.zeros((0, 4 * d * n))
            h = np.zeros((0, 1))

            H = np.zeros((4 * d * n, 4 * d * n))
            for i in range(0, n):
                Ai1, bi1 = calcAb_i1(i, n, d, dt[i], self.points[i], self.points[i + 1])
                Aeq = np.concatenate((Aeq, Ai1), axis=0)
                beq = np.concatenate((beq, bi1), axis=0)

                # Corridor Constraints
                Gi, hi = addCorridorConstraints(dt[i], self.points[i], self.points[i + 1], 1,
                                                np.sqrt(self.eps ** 2 / 3), d, n, i)
                G = np.concatenate((G, Gi), axis=0)
                h = np.concatenate((h, hi), axis=0)

                H = H + calcH_i(i, n, d, dt[i])
                if i < (n - 1):
                    for k in range(1, 5):
                        Ai2k, bi2k = calcAb_i2k(i, k, n, d, dt[i])
                        Aeq = np.concatenate((Aeq, Ai2k), axis=0)
                        beq = np.concatenate((beq, bi2k), axis=0)

            Ag, bg = goalStopConstraint(i, 2, n, d, dt[i])
            Aeq = np.concatenate((Aeq, Ag), axis=0)
            beq = np.concatenate((beq, bg), axis=0)

            QPResults = cvxopt.solvers.qp(cvxopt.matrix(H), cvxopt.matrix(np.zeros((4 * d * n, 1))), cvxopt.matrix(G),
                                          cvxopt.matrix(h), cvxopt.matrix(Aeq), cvxopt.matrix(beq))

            ind = np.arange(0, 4 * d * n, 4)
            allcoeffs = np.array(QPResults['x'])
            xcoeff = allcoeffs[ind]
            ycoeff = allcoeffs[ind + 1]
            zcoeff = allcoeffs[ind + 2]
            yawcoeff = allcoeffs[ind + 3]
            # rearrange to coefficents so each row is the nth polynomial, and each column is the dth coefficient
            self.xcoeff = np.reshape(xcoeff, (n, d))
            self.ycoeff = np.reshape(ycoeff, (n, d))
            self.zcoeff = np.reshape(zcoeff, (n, d))
            self.yawcoeff = np.reshape(yawcoeff, (n, d))

            timeat = np.zeros((n + 1, 1))
            for i in range(1, n + 1):
                timeat[i] = timeat[i - 1] + dt[i - 1]

            self.tlist = timeat[1:]

            print("WorldTraj setup done!")

    def update(self, t):
        """
        Given the present time, return the desired flat output and derivatives.

        Inputs
            t, time, s
        Outputs
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s
        """
        q = np.zeros((3,))
        q_dot = np.zeros((3,))
        q_ddot = np.zeros((3,))
        q_dddot = np.zeros((3,))
        q_ddddot = np.zeros((3,))
        yaw = 0
        yaw_dot = 0

        # if the current time is less than the final time of the first segment
        if self.xcoeff is None:
            print("Invalid Path")
        else:
            if self.tlist[-1] < t or np.isinf(t):
                # if over max time, shoot for the goal
                q = self.goal
            # if the current time is not past the goal time
            else:
                ind = np.argwhere(self.tlist > t)[0][0]
                if ind > 0:
                    self.oldTime = self.tlist[ind - 1]
                else:
                    self.oldTime = 0

                xc = self.xcoeff[ind, :]
                yc = self.ycoeff[ind, :]
                zc = self.zcoeff[ind, :]
                yawc = self.yawcoeff[ind, :]

                x, xd, xdd, xddd, xdddd = calcSnapPoly(xc, t - self.oldTime)
                y, yd, ydd, yddd, ydddd = calcSnapPoly(yc, t - self.oldTime)
                z, zd, zdd, zddd, zdddd = calcSnapPoly(zc, t - self.oldTime)
                yaw, yaw_dot, yawdd, yawddd, yawdddd = calcSnapPoly(yawc, t - self.oldTime)

                q = np.array([x, y, z]).flatten()
                q_dot = np.array([xd, yd, zd]).flatten()
                q_ddot = np.array([xdd, ydd, zdd]).flatten()
                q_dddot = np.array([xddd, yddd, zddd]).flatten()
                q_ddddot = np.array([xdddd, ydddd, zdddd]).flatten()

        flat_output = {'x': q, 'x_dot': q_dot, 'x_ddot': q_ddot, 'x_dddot': q_dddot, 'x_ddddot': q_ddddot,
                       'yaw': yaw, 'yaw_dot': yaw_dot}
        return flat_output


def dougPeuck(points, margin):
    farthest, fdist, f_ind = furthestPoint(points[0, :], points[-1, :], points)
    # if all the points are within margin of the line between the start and end
    # just keep the start and end
    if fdist < margin:
        return np.concatenate(([points[0, :]], [points[-1, :]]), axis=0)
    else:
        left = dougPeuck(points[0:f_ind + 1, :], margin)
        right = dougPeuck(points[f_ind:-1, :], margin)
        return np.concatenate((left[0:-1, :], right), axis=0)


def furthestPoint(start, goal, pointlist):
    """
    Takes 2 points (start and goal) and returns the furthest point (in pointlist) from the line between them
    """

    # base of the triangle
    b = np.linalg.norm(start - goal)
    furthest = 0
    fdist = 0
    f_ind = 0
    dist = 0

    for i in range(1, len(pointlist) - 1):
        point = pointlist[i, :]
        # area of triangle
        a = np.linalg.norm(np.cross((goal - start), (point - start)))
        # height of the triangle
        dist = a / b
        if dist > fdist:
            furthest = point
            fdist = dist
            f_ind = i
    return furthest, fdist, f_ind


def calcH_i(i, n, d, dt_i):
    """
    integral of snap squared over ith polynomial, aka cost
    i:index of polynomial
    n:number of polynomials (num waypoints-1)
    d:terms per polynomial (8, for min snap)
    dt_i: delta t for the ith segment
    """
    H_i = np.zeros((4 * d * n, 4 * d * n))
    coeffs = np.zeros((1, d - 4))
    j = np.arange(4, d)
    coeffs = ((dt_i ** ((2 * j) - 7)) * ((factorial(j)) / factorial(j - 4)))
    temp = np.array([0, 0, 0, 0])
    coeffs = np.concatenate((temp, coeffs))
    coeffs = np.repeat(coeffs, 4)  # one copy for each state variable
    startind = i * 4 * d
    endind = startind + len(coeffs)
    H_i[startind:endind, startind:endind] = np.diag(coeffs)
    return H_i


def calcAb_i1(i, n, d, dt_i, w_i, w_ip1):
    """
    Position constraints for minsnap QP
     i: Index of polynomial (which eq you are on)
     n: total number of polynomials
     d: # terms per polynomial
     dt_i: duration of current time segment
     w_i: waypoint at start of current segment
     w_ip1: waypoint at end of current segment
    :return: A and b for Ax=b constraint of QP
    """
    Ai_1 = np.zeros((8, 4 * d * n))  # 2xlen(state)=8

    # set beginning and end of polynomial to hit the corresponding waypoints
    bi_1 = np.array([[w_i[0]], [w_i[1]], [w_i[2]], [0],
                     [w_ip1[0]], [w_ip1[1]], [w_ip1[2]], [0]])

    startind = i * 4 * d
    # wi=sig_i,0
    Ai_1[0:4, startind:startind + 4] = np.diag([1, 1, 1, 1])
    # wip1= polynomial at dt
    j = np.arange(0, d)
    dt = dt_i ** j
    sigind = np.arange(startind, startind + 4 * d, 4)
    Ai_1[4, sigind] = dt
    Ai_1[5, sigind + 1] = dt
    Ai_1[6, sigind + 2] = dt
    Ai_1[7, sigind + 3] = dt

    return Ai_1, bi_1


def calcAb_i2k(i, k, n, d, dt_i):
    """
    derivative continuity constraints up to kth order derivative
     i: index of polynomial (which segment you are in)
     k: Order of derivative being taken
     n: Total # polynomials
     d: # terms per polynomial
     dt_i: duration of the ith segment
    :return: A and b for Ax = b equality constraint of QP
    """

    Ai_2k = np.zeros((4, 4 * d * n))
    bi_2k = np.zeros((4, 1))

    j = np.arange(k, d)
    coeff = (dt_i ** (j - k)) * (factorial(j) / factorial(j - k))
    startind = (4 * d * i) + (4 * k)
    endind = startind + 4 * len(j)
    iind = np.arange(startind, endind, 4)
    Ai_2k[0, iind] = coeff
    Ai_2k[1, iind + 1] = coeff
    Ai_2k[2, iind + 2] = coeff
    Ai_2k[3, iind + 3] = coeff

    # i+1 part of constraint
    startind = (4 * d * (i + 1)) + (4 * k)
    ip1ind = np.arange(startind, startind + 4)
    Ai_2k[:, ip1ind] = -factorial(k) * np.diag([1, 1, 1, 1])
    return Ai_2k, bi_2k


def addCorridorConstraints(dt, wi, wip1, num_subpoints, margin, d, n, i):
    """
    Returns num_subpoints corridor constraints of the form Gx<=h
    dt: The delta time for the whole time interval
    wi: waypoint i
    wip1: waypoint i+1
    num_subpoints: number of intermediate points between wi and wi+1 to add corridor constraints on
    margin: how wide the corridor is
    d: number of coefficients in each polynomial
    n: number of polynomials
    :return: G,h of Gx<=h
    """
    # 6=x,y,z* 2(for both ends of absolute value)
    G = np.zeros((6 * num_subpoints, 4 * d * n))
    h = np.zeros((6 * num_subpoints, 1))

    # the +2 and the 1:-1 are to get rid of the endpoints, which are already constrained
    dt_sub = np.linspace(0, dt, num_subpoints + 2)[1:-1]
    w_sub = np.linspace(wi, wip1, num_subpoints + 2)[1:-1]

    # Polynomial exponents
    j = np.arange(0, d)

    startind = i * 4 * d
    for i in range(0, num_subpoints):
        offset = np.array([[w_sub[i][0]], [w_sub[i][1]], [w_sub[i][2]]])
        h[6 * i:6 * i + 3] = margin + offset
        h[6 * i + 3:6 * i + 6] = margin - offset

        # wip1= polynomial at dt

        T = dt_sub[i] ** j
        sigind = np.arange(startind, startind + 4 * d, 4)
        # Positive half of abs <=
        G[6 * i, sigind] = T
        G[6 * i + 1, sigind + 1] = T
        G[6 * i + 2, sigind + 2] = T
        # negative half
        G[6 * i + 3, sigind] = -T
        G[6 * i + 4, sigind + 1] = -T
        G[6 * i + 5, sigind + 2] = -T
    return G, h


def goalStopConstraint(i, k, n, d, dt_i):
    """
     i: index of polynomial (which segment you are in)
     k: Order of derivative being taken
     n: Total # polynomials
     d: # terms per polynomial
     dt_i: duration of the ith segment
    :return: Ag,bg, equality constraints for goal which set the final velocity and acceleration to 0
    """
    Ag = np.zeros((k * 4, 4 * d * n))
    bg = np.zeros((k * 4, 1))
    startind = i * 4 * d

    # Velocity (first derivative)
    j = np.arange(0, d - 1)
    dt = np.concatenate([[0], (dt_i ** j) * (j + 1)])
    sigind = np.arange(startind, startind + 4 * d, 4)
    Ag[0, sigind] = dt
    Ag[1, sigind + 1] = dt
    Ag[2, sigind + 2] = dt
    Ag[3, sigind + 3] = dt

    # Acceleration (second derivative)
    j = np.arange(0, d - 2)
    dt = np.concatenate([[0], [0], (dt_i ** j) * factorial(j + 1)])
    Ag[4, sigind] = dt
    Ag[5, sigind + 1] = dt
    Ag[6, sigind + 2] = dt
    Ag[7, sigind + 3] = dt

    return Ag, bg


def calcSnapPoly(coeff, t):
    """
    returns x through its 4th derivative, evaluated at time t with the provided polynomial coefficients (coeff)
    """
    x = coeff[0] + coeff[1] * t + coeff[2] * (t ** 2) + coeff[3] * (t ** 3) + \
        coeff[4] * (t ** 4) + coeff[5] * (t ** 5) + coeff[6] * (t ** 6) + coeff[7] * (t ** 7)
    xd = coeff[1] + 2 * coeff[2] * (t) + 3 * coeff[3] * (t ** 2) + \
         4 * coeff[4] * (t ** 3) + 5 * coeff[5] * (t ** 4) + 6 * coeff[6] * (t ** 5) + 7 * coeff[7] * (t ** 6)
    xdd = 2 * coeff[2] + 6 * coeff[3] * (t) + 12 * coeff[4] * (t ** 2) + \
          20 * coeff[5] * (t ** 3) + 30 * coeff[6] * (t ** 4) + 42 * coeff[7] * (t ** 5)
    xddd = 6 * coeff[3] + 24 * coeff[4] * (t) + 60 * coeff[5] * (t ** 2) + 120 * coeff[6] * (t ** 3) + 210 * coeff[
        7] * (t ** 4)
    xdddd = 24 * coeff[4] + 120 * coeff[5] * (t) + 360 * coeff[6] * (t ** 2) + 840 * coeff[7] * (t ** 3)

    return x, xd, xdd, xddd, xdddd
