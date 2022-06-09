import swift
import roboticstoolbox as rtb
import spatialgeometry as sg
import spatialmath as sm
import qpsolvers as qp
import numpy as np
import math


def arm(r, bTep):

    bTe = r.fkine(r.q, include_base=False, fast=True)

    eTep = np.linalg.inv(bTe) @ bTep

    # Spatial error
    et = np.sum(np.abs(eTep[:3, -1]))

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

    Ain_torso, bin_torso = r.joint_velocity_damper(0.0, 0.05, r.n)
    Ain[2, 2] = Ain_torso[2, 2]
    bin[2] = bin_torso[2]

    # Linear component of objective function: the manipulability Jacobian
    c = np.concatenate(
        (np.zeros(2), -r.jacobm(start=r.links[3]).reshape((r.n - 2,)), np.zeros(6))
    )

    # The lower and upper bounds on the joint velocity and slack variable
    lb = -np.r_[r.qdlim[: r.n], 1000000 * np.ones(6)]
    ub = np.r_[r.qdlim[: r.n], 1000000 * np.ones(6)]

    # Solve for the joint velocities dq
    qd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub)
    qd = qd[: r.n]
    
    if et < 0.05:
        return True, qd
    else:
        return False, qd


def base(r, b_ratio, wTb_cumulative_lengths):
    LOOKAHEAD_DISTACE = 0.5
    lookahead_ratio = LOOKAHEAD_DISTACE / wTb_cumulative_lengths[-1]

    wTb_lin = interpolate_path(b_ratio, wTb_cumulative_lengths)
    bTbp_lin = np.linalg.inv(r._base.A) @ sm.SE2(wTb_lin).SE3().A
    v = np.linalg.norm(bTbp_lin[0:2, 3]) * np.sign(bTbp_lin[0, 3]) * 5
    v = max(min(v, r.qdlim[1]), -r.qdlim[1])

    wTb_ang = interpolate_path(b_ratio + lookahead_ratio*np.sign(bTbp_lin[0, 3]), wTb_cumulative_lengths)
    bTbp_ang = np.linalg.inv(r._base.A) @ sm.SE2(wTb_ang).SE3().A
    kappa = (2.0 * bTbp_ang[1, 3]) / (np.square(bTbp_ang[0, 3]) + np.square(bTbp_ang[1, 3]))
    if np.isnan(kappa):
        kappa = 0
    omega = v * kappa

    return np.array([omega, v])


def interpolate_path(b_ratio, wTb_cumulative_lengths):
    b_ratio = min(max(0.0, b_ratio), 1.0)
    wTb_total_length = wTb_cumulative_lengths[-1]
    for i in range(len(wTb_cumulative_lengths) - 1):
        if wTb_cumulative_lengths[i + 1] / wTb_total_length >= b_ratio:

            ratio_a = wTb_cumulative_lengths[i] / wTb_total_length
            ratio_b = wTb_cumulative_lengths[i + 1] / wTb_total_length

            ratio = (b_ratio - ratio_a) / (ratio_b - ratio_a)

            return wTb_waypoints[i] * (1 - ratio) + wTb_waypoints[i + 1] * ratio

env = swift.Swift()
env.launch(realtime=True)

ee_goal = sg.Axes(0.1)
env.add(ee_goal)
b_goal = sg.Axes(0.1)
env.add(b_goal)

fetch = rtb.models.Fetch()
fetch.q = fetch.qr
env.add(fetch)

arrived = False
dt = 0.025

# Behind
bTe_handover = np.array([0.9, 0.0, 0.8])
bTe_tucked = np.array([0.3, 0, 0.6])

wTb_waypoints = [np.array([0.0, 0.0]),
                 np.array([5, 0.0]),
                 np.array([5, 5]),
                 np.array([0.0, 5])]

wTb_cumulative_lengths = [0.0]
for i in range(len(wTb_waypoints) - 1):
    l = np.linalg.norm(wTb_waypoints[i + 1] - wTb_waypoints[i])
    wTb_cumulative_lengths += [wTb_cumulative_lengths[i] + l]

b_ratio = 0.0
wTb = interpolate_path(b_ratio, wTb_cumulative_lengths)
b_goal.base = sm.SE2(wTb).SE3()

ee_ratio = 0.0 # 0: Tucked, 1: Handovver
bTep = sm.SE3(bTe_tucked * ee_ratio + bTe_handover * (1 - ee_ratio)) * sm.SE3.Rz(np.pi / 2 * (1 - ee_ratio))
env.set_camera_pose([-2, 3, 0.7], [-2, 0.0, 0.5])
ee_goal.base = fetch.base * bTep

env.step()
t=0

while True:

    arrived, fetch.qd = arm(fetch, bTep.A)
    base_qd = base(fetch, b_ratio, wTb_cumulative_lengths)
    fetch.qd[0:2] = base_qd
    env.step(dt)

    # Reset bases
    base_new = fetch.fkine(fetch._q, end=fetch.links[2], fast=True)
    fetch._base.A[:] = base_new
    fetch.q[:2] = 0

    t += 0.01
    ee_ratio = 0.5 * np.cos(t) + 0.5
    bTep = sm.SE3(bTe_tucked * (1 - ee_ratio) + bTe_handover * ee_ratio) * sm.SE3.Rz(np.pi / 2 * (1 - ee_ratio))
    ee_goal._base = (fetch.base * bTep).A

    b_ratio = -0.5 * np.cos(t) + 0.5
    wTb = interpolate_path(b_ratio, wTb_cumulative_lengths)
    b_goal.base = sm.SE2(wTb).SE3()

env.hold()