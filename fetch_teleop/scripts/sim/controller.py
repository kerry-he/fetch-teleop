from cmath import inf
import roboticstoolbox as rtb
import spatialmath as sm
import qpsolvers as qp
import numpy as np

def interpolate_path(b_ratio, wTb_cumulative_lengths, wTb_waypoints):
    b_ratio = min(max(0.0, b_ratio), 1.0)
    wTb_total_length = wTb_cumulative_lengths[-1]
    for i in range(len(wTb_cumulative_lengths) - 1):
        if wTb_cumulative_lengths[i + 1] / wTb_total_length >= b_ratio:

            ratio_a = wTb_cumulative_lengths[i] / wTb_total_length
            ratio_b = wTb_cumulative_lengths[i + 1] / wTb_total_length

            ratio = (b_ratio - ratio_a) / (ratio_b - ratio_a)

            return wTb_waypoints[i] * (1 - ratio) + wTb_waypoints[i + 1] * ratio


def arm(r, bTep):

    bTe = r.fkine(r.q, include_base=False, fast=True)

    eTep = np.linalg.inv(bTe) @ bTep

    # Spatial error
    et = np.sum(np.abs(eTep[:3, -1])) + 1e-5

    # Gain term (lambda) for control minimisation
    Y = 0.01

    # Quadratic component of objective function
    Q = np.eye(r.n + 6)

    # Joint velocity component of Q
    Q[: r.n, : r.n] *= Y
    Q[:2, :2] *= 1.0 / et

    # Slack component of Q
    Q[r.n :, r.n :] = (1.0 / et) * np.eye(6)

    v, _ = rtb.p_servo(bTe, bTep, 3.0)

    v[3:] *= 1.3

    v_curr = np.linalg.norm(v)

    # The equality contraints
    Aeq = np.c_[r.jacobe(r.q, fast=True), np.eye(6)]
    beq = v.reshape((6,))

    Aeq_arm = np.c_[np.eye(3), np.zeros((3, 7)), np.zeros((3, 6))]
    beq_arm = np.zeros((3,))
    beq_arm[-1] = r.qd[2]

    Aeq = np.r_[Aeq, Aeq_arm]
    beq = np.r_[beq, beq_arm]

    # The inequality constraints for joint limit avoidance
    Ain = np.zeros((r.n + 6, r.n + 6))
    bin = np.zeros(r.n + 6)

    # The minimum angle (in radians) in which the joint is allowed to approach
    # to its limit
    ps = 0.1

    # The influence angle (in radians) in which the velocity damper
    # becomes active
    pi = 0.9

    # Form the joint limit velocity damper
    Ain[: r.n, : r.n], bin[: r.n] = r.joint_velocity_damper(ps, pi, r.n)

    # Linear component of objective function: the manipulability Jacobian
    c = np.concatenate(
        (np.zeros(2), -r.jacobm(start=r.links[3]).reshape((r.n - 2,)), np.zeros(6))
    )

    # The lower and upper bounds on the joint velocity and slack variable
    lb = -np.r_[r.qdlim[: r.n] / 2, 10000000000 * np.ones(6)]
    ub = np.r_[r.qdlim[: r.n] / 2, 10000000000 * np.ones(6)]

    # Solve for the joint velocities dq
    try:
        qd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub)
        qd = qd[: r.n]
    except:
        qd = np.zeros(r.n)
    
    return qd


HYST_B_FORWARDS = True


def base(r, v, omega, b_ratio, wTb_cumulative_lengths, wTb_waypoints, segment):
    LOOKAHEAD_DISTACE = 0.5
    lookahead_ratio = LOOKAHEAD_DISTACE / wTb_cumulative_lengths[-1]

    STOP_DISTANCE = 0.5
    stop_ratio = STOP_DISTANCE / wTb_cumulative_lengths[-1]

    v = max(min(v, r.qdlim[1]), -r.qdlim[1])
    omega = max(min(omega, r.qdlim[0]), -r.qdlim[0])

    # Determine direction of travel
    direction = np.sign(np.dot(r._base.A[0:2, 0], wTb_waypoints[segment + 1] - wTb_waypoints[segment])) * np.sign(v)

    # Logic to slow down at path endpoints
    if direction > 0.0 and (1 - b_ratio) < stop_ratio:
        v *= max((1 - b_ratio) / stop_ratio, 0.0)
    elif direction < 0.0 and b_ratio < stop_ratio:
        v *= max(b_ratio / stop_ratio, 0.0)


    # Logic to interpolate lookahead point past path waypoints
    if b_ratio + lookahead_ratio*direction > 1.0:
        wTb_ang = interpolate_path(b_ratio, wTb_cumulative_lengths, wTb_waypoints)
        u = wTb_waypoints[-1] - wTb_waypoints[-2]
        u *= LOOKAHEAD_DISTACE / np.linalg.norm(v)
        wTb_ang += u
    elif b_ratio + lookahead_ratio*direction < 0.0:
        wTb_ang = interpolate_path(b_ratio, wTb_cumulative_lengths, wTb_waypoints)
        u = wTb_waypoints[0] - wTb_waypoints[1]
        u *= LOOKAHEAD_DISTACE / np.linalg.norm(v)
        wTb_ang += u
    else:
        wTb_ang = interpolate_path(b_ratio + lookahead_ratio*direction, wTb_cumulative_lengths, wTb_waypoints)

    
    # Pure pursuit calculation for steering
    bTbp_ang = np.linalg.inv(r._base.A) @ sm.SE2(wTb_ang).SE3().A
    kappa = (2.0 * bTbp_ang[1, 3]) / (np.square(bTbp_ang[0, 3]) + np.square(bTbp_ang[1, 3]) + 1e-5)
    if np.isnan(kappa):
        kappa = 0.0
    # omega += abs(v) * kappa

    # if reverse:
    omega += v * kappa
    return np.array([omega, v])
    # else:

    #     angle_to_lookahead = np.arctan2(bTbp_ang[1, 3], bTbp_ang[0, 3])

    #     global HYST_B_FORWARDS
    #     if HYST_B_FORWARDS:
    #         if abs(angle_to_lookahead) > np.deg2rad(90):
    #             HYST_B_FORWARDS = False
    #             omega = v
    #             v = 0
    #     elif abs(angle_to_lookahead) < np.deg2rad(15):
    #         HYST_B_FORWARDS = True
    #     else:
    #         omega = v
    #         v = 0 

    #     return np.array([omega, abs(v)])


def closest_point(r, wTb_cumulative_lengths, wTb_waypoints):
    r_pos = r._base.A[0:2, 3]

    d_min = np.Inf
    s = 0
    segment = 0

    for i in range(len(wTb_waypoints) - 1):
        u = wTb_waypoints[i] - r_pos
        v = wTb_waypoints[i + 1] - wTb_waypoints[i]

        t = -np.dot(v, u) / np.dot(v, v)
        t = max(min(t, 1.0), 0.0)

        d = t*t*np.dot(v, v) + np.dot(u, u) + 2*t*np.dot(v, u)
        
        if d < d_min:
            d_min = d
            s = wTb_cumulative_lengths[i] + t * (wTb_cumulative_lengths[i + 1] - wTb_cumulative_lengths[i])
            segment = i

    return s / wTb_cumulative_lengths[-1], segment


def torso(r, msg, torso_max, edit_mode):
    STOP_PC = 0.1
    stop_dist = (torso_max + 1e-5) * STOP_PC

    qd_torso = (msg.button_triangle - msg.button_cross) * msg.button_l1 / 10

    # Logic to slow down at path endpoints
    if not edit_mode:
        if qd_torso > 0.0 and (torso_max - r.q[2]) < stop_dist:
            qd_torso *= max((torso_max - r.q[2]) / stop_dist, 0.0)
        elif qd_torso < 0.0 and r.q[2] < stop_dist:
            qd_torso *= max(r.q[2] / stop_dist, 0.0)

    return qd_torso


def edit_ee_waypoint(msg, rate):
    return np.array([
        msg.axis_right_y,
        msg.axis_right_x,
        msg.button_dpad_up - msg.button_dpad_down
    ]) * rate

def edit_ee_angle_waypoint(msg, rate):
    return np.array([
        0.0,
        msg.axis_left_x,
        msg.axis_left_y
    ]) * rate

def edit_base_waypoint(r, wTb_waypoints, b_ratio):
    # Calculate change in waypoint
    u0 = wTb_waypoints[0] - wTb_waypoints[1]
    u0 /= np.linalg.norm(u0)
    uf = wTb_waypoints[-1] - wTb_waypoints[-2]
    uf /= np.linalg.norm(uf)

    if b_ratio < 0.5:
        delta = r._base.A[0:2, -1] - wTb_waypoints[0]
        angle = np.arctan2(delta[1], delta[0]) - np.arctan2(u0[1], u0[0])
    else:
        delta = r._base.A[0:2, -1] - wTb_waypoints[-1]
        angle = np.arctan2(delta[1], delta[0]) - np.arctan2(uf[1], uf[0])

    length = np.linalg.norm(delta)

    c = np.cos(angle)
    s = np.sin(angle)
    T = length * np.array([[c, -s], [s, c]])

    wTb_waypoints[0] += np.matmul(T, u0)
    wTb_waypoints[-1] += np.matmul(T, uf)

    return wTb_waypoints

def edit_ee_tucked(bRe_tucked, msg, rate):
    bRe_tucked += np.array([
        0.0,
        msg.button_dpad_left - msg.button_dpad_right,
        0.0
    ]) * rate * 2.5

    bRe_tucked[1] = (bRe_tucked[1] + np.pi) % (2 * np.pi) - np.pi

    if bRe_tucked[1] > 0:
        bRe_tucked[1] = max(bRe_tucked[1], np.pi / 2)
    else:
        bRe_tucked[1] = min(bRe_tucked[1], -np.pi / 2)

    return bRe_tucked


def rotationToRPY(R):
    '''
    Concert a rotation matrix into the Mayavi/Vtk rotation paramaters (pitch, roll, yaw)
    '''
    def euler_from_matrix(matrix):
        """Return Euler angles (syxz) from rotation matrix for specified axis sequence.
        :Author:
          `Christoph Gohlke <http://www.lfd.uci.edu/~gohlke/>`_

        full library with coplete set of euler triplets (combinations of  s/r x-y-z) at
            http://www.lfd.uci.edu/~gohlke/code/transformations.py.html

        Note that many Euler angle triplets can describe one matrix.
        """
        # epsilon for testing whether a number is close to zero
        _EPS = np.finfo(float).eps * 5.0

        # axis sequences for Euler angles
        _NEXT_AXIS = [1, 2, 0, 1]
        firstaxis, parity, repetition, frame = (1, 1, 0, 0) # ''

        i = firstaxis
        j = _NEXT_AXIS[i+parity]
        k = _NEXT_AXIS[i-parity+1]

        M = np.array(matrix, dtype='float', copy=False)[:3, :3]
        if repetition:
            sy = np.sqrt(M[i, j]*M[i, j] + M[i, k]*M[i, k])
            if sy > _EPS:
                ax = np.arctan2( M[i, j],  M[i, k])
                ay = np.arctan2( sy,       M[i, i])
                az = np.arctan2( M[j, i], -M[k, i])
            else:
                ax = np.arctan2(-M[j, k],  M[j, j])
                ay = np.arctan2( sy,       M[i, i])
                az = 0.0
        else:
            cy = np.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
            if cy > _EPS:
                ax = np.arctan2( M[k, j],  M[k, k])
                ay = np.arctan2(-M[k, i],  cy)
                az = np.arctan2( M[j, i],  M[i, i])
            else:
                ax = np.arctan2(-M[j, k],  M[j, j])
                ay = np.arctan2(-M[k, i],  cy)
                az = 0.0

        if parity:
            ax, ay, az = -ax, -ay, -az
        if frame:
            ax, az = az, ax
        return ax, ay, az
    r_yxz = np.array(euler_from_matrix(R))
    r_xyz = r_yxz[[1, 2, 0]]
    return r_xyz