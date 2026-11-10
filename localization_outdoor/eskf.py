"""15-state inertial error filter. Frames: ENU world, IMU origin, body axes.

GNSS observations constrain horizontal antenna position/velocity only. The
unobserved vertical state is deliberately not advertised as map elevation.
"""

from dataclasses import dataclass
import copy
import math
import numpy as np


def skew(v):
    x, y, z = v
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def exp_rotation(v):
    angle = np.linalg.norm(v)
    k = skew(v)
    if angle < 1e-8:
        return np.eye(3) + k + 0.5 * k @ k
    return (
        np.eye(3)
        + math.sin(angle) / angle * k
        + (1 - math.cos(angle)) / angle**2 * k @ k
    )


def yaw_jacobian(rotation):
    horizontal = rotation[0, 0] ** 2 + rotation[1, 0] ** 2
    if horizontal < 1e-12:
        return np.array([0.0, 1e6, 1e6])
    return np.array([0.0, rotation[2, 1], rotation[2, 2]]) / horizontal


@dataclass
class State:
    t: float
    p: np.ndarray
    v: np.ndarray
    R: np.ndarray
    bg: np.ndarray
    ba: np.ndarray
    P: np.ndarray
    acceleration: np.ndarray
    gyro: np.ndarray


class ErrorStateFilter:
    def __init__(self, state, noise, arm, history_s=2.0):
        self.state = state
        self.noise = noise
        self.arm = np.asarray(arm, dtype=float)
        self.history_s = history_s
        self.base = copy.deepcopy(state)
        self.events = []
        self.checkpoints = []
        self.replays = 0
        self.dropped = 0

    def propagate(self, t):
        s = self.state
        dt = t - s.t
        if dt < -1e-8:
            raise ValueError("Filter time reversed")
        if dt <= 0:
            return
        if dt > 0.5:
            raise ValueError("Inertial continuity lost")
        w, a = s.gyro - s.bg, s.acceleration - s.ba
        mid = s.R @ exp_rotation(w * dt / 2)
        world_a = mid @ a + np.array([0.0, 0.0, -9.80665])
        s.p += s.v * dt + 0.5 * world_a * dt**2
        s.v += world_a * dt
        s.R = s.R @ exp_rotation(w * dt)
        F = np.zeros((15, 15))
        F[:3, 3:6] = np.eye(3)
        F[3:6, 6:9] = -mid @ skew(a)
        F[3:6, 12:15] = -mid
        F[6:9, 6:9] = -skew(w)
        F[6:9, 9:12] = -np.eye(3)
        phi = np.eye(15) + F * dt + 0.5 * (F @ F) * dt**2
        Q = np.zeros((15, 15))
        for block, key in (
            (3, "accel_noise"),
            (6, "gyro_noise"),
            (9, "gyro_bias_walk"),
            (12, "accel_bias_walk"),
        ):
            Q[block : block + 3, block : block + 3] = np.eye(3) * self.noise[key] ** 2
        # Integrated white acceleration: keep position/velocity cross covariance.
        Q[:3, :3] = Q[3:6, 3:6] * dt**2 / 3
        Q[:3, 3:6] = Q[3:6, 3:6] * dt / 2
        Q[3:6, :3] = Q[:3, 3:6].T
        s.P = phi @ s.P @ phi.T + Q * dt
        s.P = (s.P + s.P.T) / 2
        s.t = t

    def correct(self, residual, H, V):
        s = self.state
        K = np.linalg.solve(H @ s.P @ H.T + V, H @ s.P).T
        dx = K @ residual
        s.p += dx[:3]
        s.v += dx[3:6]
        s.R = s.R @ exp_rotation(dx[6:9])
        s.bg += dx[9:12]
        s.ba += dx[12:15]
        A = np.eye(15) - K @ H
        s.P = A @ s.P @ A.T + K @ V @ K.T
        reset = np.eye(15)
        reset[6:9, 6:9] -= 0.5 * skew(dx[6:9])
        s.P = reset @ s.P @ reset.T
        s.P = (s.P + s.P.T) / 2

    def apply(self, t, kind, data):
        self.propagate(t)
        s = self.state
        if kind == "inertial":
            s.acceleration, s.gyro = (np.array(v, dtype=float) for v in data)
        elif kind == "heading":
            observed, variance = data
            # Only use magnetic yaw; gyro carries roll/pitch dynamically.
            yaw = math.atan2(s.R[1, 0], s.R[0, 0])
            error = math.atan2(math.sin(observed - yaw), math.cos(observed - yaw))
            if abs(error) <= math.radians(30) and np.hypot(s.R[0, 0], s.R[1, 0]) > 1e-6:
                H = np.zeros((1, 15))
                H[0, 6:9] = yaw_jacobian(s.R)
                self.correct(np.array([error]), H, np.array([[variance]]))
        else:
            position, velocity, hacc, sacc, reset = data
            arm_world = s.R @ self.arm
            arm_velocity = s.R @ np.cross(s.gyro - s.bg, self.arm)
            if reset:
                s.p[:2] = np.asarray(position) - arm_world[:2]
                s.v[:2] = np.asarray(velocity) - arm_velocity[:2]
                # Reset observed blocks without leaving stale cross covariance.
                for ids, var in (([0, 1], hacc**2), ([3, 4], sacc**2)):
                    s.P[ids, :] = 0.0
                    s.P[:, ids] = 0.0
                    s.P[np.ix_(ids, ids)] = np.eye(2) * var
            else:
                H = np.zeros((4, 15))
                H[:2, :3] = np.eye(3)[:2]
                H[:2, 6:9] = (-s.R @ skew(self.arm))[:2]
                H[2:, 3:6] = np.eye(3)[:2]
                H[2:, 6:9] = (-s.R @ skew(np.cross(s.gyro - s.bg, self.arm)))[:2]
                H[2:, 9:12] = (s.R @ skew(self.arm))[:2]
                residual = np.r_[
                    np.asarray(position) - (s.p + arm_world)[:2],
                    np.asarray(velocity) - (s.v + arm_velocity)[:2],
                ]
                self.correct(residual, H, np.diag([hacc**2] * 2 + [sacc**2] * 2))

    def insert(self, t, kind, data):
        if t < self.base.t or t < self.state.t - self.history_s:
            self.dropped += 1
            return False
        delayed = t < self.state.t
        event = (t, kind, copy.deepcopy(data))
        # Stable ordering keeps samples first, observations second at equal times.
        index = next(
            (i for i, row in enumerate(self.events) if row[0] > t), len(self.events)
        )
        self.events.insert(index, event)
        if delayed:
            self.replays += 1
            self.state = copy.deepcopy(
                self.checkpoints[index - 1] if index else self.base
            )
            del self.checkpoints[index:]
            for at, tag, value in self.events[index:]:
                self.apply(at, tag, value)
                self.checkpoints.append(copy.deepcopy(self.state))
        else:
            self.apply(t, kind, data)
            self.checkpoints.append(copy.deepcopy(self.state))
        cutoff = self.state.t - self.history_s
        count = next(
            (i for i, row in enumerate(self.events) if row[0] >= cutoff),
            len(self.events),
        )
        if count:
            self.base = self.checkpoints[count - 1]
            del self.events[:count]
            del self.checkpoints[:count]
        return True
