"""Torque-limited four-wheel steering; the free base is moved only by contacts."""

import numpy as np


class MobileBase:
    MAX_SPEED = 0.08
    MAX_YAW_RATE = 0.12

    def __init__(self, sim):
        self.sim = sim
        m = sim.model
        self.command = np.zeros(3)
        self.velocity = np.zeros(3)
        self.steer = []
        self.drive = []
        self.offsets = []
        for prefix, location in [
            (111, "lwheel_front"),
            (131, "rwheel_front"),
            (141, "rwheel_rear"),
            (121, "lwheel_rear"),
        ]:
            stem = f"chassis_{location}"
            self.steer.append(m.actuator(f"idx{prefix}_{stem}_joint1_position").id)
            self.drive.append(m.actuator(f"idx{prefix+1}_{stem}_joint2_position").id)
            self.offsets.append(m.body_pos[m.body(stem + "_link1").id, :2].copy())
        self.steer = np.array(self.steer)
        self.drive = np.array(self.drive)
        self.offsets = np.array(self.offsets)
        self.qadr = m.joint("g2_mobile_base").qposadr[0]
        self.parking_pose = None

    @staticmethod
    def validate(velocity):
        v = np.asarray(velocity, dtype=float)
        if v.shape != (3,) or not np.isfinite(v).all():
            raise ValueError("Base command is world [vx, vy, yaw_rate]")
        if (
            np.linalg.norm(v[:2]) > MobileBase.MAX_SPEED + 1e-9
            or abs(v[2]) > MobileBase.MAX_YAW_RATE
        ):
            raise ValueError("Base velocity exceeds 0.08 m/s or 0.12 rad/s")
        return v

    def pose(self):
        # The hook runs before mj_forward; derived xpos/xmat may still describe
        # the previous integration step (or be unset at episode initialization).
        q = self.sim.data.qpos[self.qadr : self.qadr + 7]
        w, x, y, z = q[3:]
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return np.array([*q[:2], yaw])

    def update(self):
        s = self.sim
        dt = s.model.opt.timestep
        pose = self.pose()
        command = self.command.copy()
        if np.linalg.norm(command) > 1e-8:
            self.parking_pose = None
        elif self.parking_pose is None:
            if np.linalg.norm(self.velocity) < 1e-5:
                self.parking_pose = pose.copy()
        else:
            error = self.parking_pose - pose
            error[2] = (error[2] + np.pi) % (2 * np.pi) - np.pi
            if np.linalg.norm(error[:2]) > 0.002:
                base_dof = s.model.joint("g2_mobile_base").dofadr[0]
                command[:2] = (
                    error[:2] * 1.5 - 0.8 * s.data.qvel[base_dof : base_dof + 2]
                )
                command[:2] *= min(1.0, 0.025 / max(np.linalg.norm(command[:2]), 1e-9))
            if abs(error[2]) > 0.004:
                command[2] = np.clip(error[2] * 1.5, -0.04, 0.04)
        self.velocity += np.clip(
            command - self.velocity,
            -np.array([0.04, 0.04, 0.10]) * dt,
            np.array([0.04, 0.04, 0.10]) * dt,
        )
        yaw = pose[2]
        c, sn = np.cos(yaw), np.sin(yaw)
        vx, vy = np.array([[c, sn], [-sn, c]]) @ self.velocity[:2]
        omega = self.velocity[2]
        for steer, drive, (x, y) in zip(self.steer, self.drive, self.offsets):
            wheel_v = np.array([vx - omega * y, vy + omega * x])
            speed = np.linalg.norm(wheel_v)
            if speed < 1e-5:
                continue
            theta = np.arctan2(wheel_v[1], wheel_v[0])
            current = s.data.qpos[s.qids[steer]]
            choices = [
                (theta + k * np.pi, speed * (-1) ** k)
                for k in range(-2, 3)
                if -2.969 < theta + k * np.pi < 2.969
            ]
            angle, signed_speed = min(choices, key=lambda item: abs(item[0] - current))
            s.target[steer] += np.clip(angle - s.target[steer], -dt, dt)
            # Steering completes before rolling; no lateral ground-force shortcut.
            alignment = max(0.0, 1.0 - abs(angle - current) / 0.12)
            s.target[drive] += signed_speed / 0.07 * alignment * dt

    def stop(self):
        self.command[:] = 0.0
