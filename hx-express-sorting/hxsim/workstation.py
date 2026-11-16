"""Operations and scoring in the shared index/MuJoCo workstation."""
from collections import deque
import hashlib
import json
import time
import copy
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from hxsim.runtime import G2Simulation, ROOT
from hxsim.mobile import MobileBase
INITIAL_POSITION = np.array([-0.1, -0.32, 1.13])
INITIAL_ROTATION = Rotation.from_euler('x', -np.pi / 2).as_matrix()
PERIOD = 0.2
MAX_EPISODE_STEPS = 20000

class WorkstationSimulation(G2Simulation):

    def __init__(self):
        super().__init__(ROOT / 'scenes/g2_operation.xml')
        self.arm_ids = {s: np.array([self.model.joint(f'idx{o + i}_arm_{s}_joint{i + 1}').id for i in range(7)]) for (s, o) in [('l', 21), ('r', 61)]}
        self.gripper_aids = {s: self.model.actuator(f'idx{o}_gripper_{s}_inner_joint1_position').id for (s, o) in [('l', 31), ('r', 71)]}
        self.version = 0
        self.primitive_steps = 0
        self.ended = False
        self.fault = None
        self.stability = deque(maxlen=101)
        self.events = []
        self.belt_running = False
        self.belt_speed = 0.08
        self.scan_records = {}
        self.loaded_records = {}
        self.support_since = {}
        self.task_mode = 'single'
        self.handling_policy = 'careful'
        self.parcels = json.loads((ROOT / 'scenes/workstation_manifest.json').read_text())['parcels']
        self.parcel_geoms = {self.model.geom('test_parcel_geom' if p['id'] == 'test_parcel' else p['id'] + '_collision').id: p['id'] for p in self.parcels}
        self.belt_aids = np.array([self.model.actuator(f'belt_drive_{i:02d}').id for i in range(30)])
        self.belt_qids = self.qids[self.belt_aids]
        self.belt_bids = np.array([self.model.body(f'belt_slat_{i:02d}').id for i in range(30)])
        self.button_gid = self.model.geom('Conveyor_StartCap').id
        self.button_qid = self.model.joint('conveyor_start_switch').qposadr[0]
        self.stop_button_gid = self.model.geom('Conveyor_EStop').id
        self.stop_button_qid = self.model.joint('conveyor_stop_switch').qposadr[0]
        self.button_gids = {self.button_gid, self.stop_button_gid}
        self.start_pressed_last = False
        self.opening_commands = {'l': 1.0, 'r': 1.0}
        self.mobile = MobileBase(self)
        (ids, q) = self.ik('r', INITIAL_POSITION, INITIAL_ROTATION, collision_check=True)
        self.data.qpos[self.model.jnt_qposadr[ids]] = q
        self.target = self.data.qpos[self.qids].copy()
        self.step(1000)
        self.background_initial = {f'P{i:03d}': self.data.body(f'P{i:03d}').xpos.copy() for i in range(1, 7)}
        self.initial_qpos = self.data.qpos.copy()

    def before_step(self):
        if hasattr(self, 'mobile'):
            self.mobile.update()
        if not hasattr(self, 'belt_aids'):
            return
        self.target[self.belt_aids] = self.belt_speed if self.belt_running else 0.0
        x = self.model.body_pos[self.belt_bids, 0] + self.data.qpos[self.belt_qids]
        for qi in self.belt_qids[x > 3.2]:
            self.data.qpos[qi] -= 3.0

    def after_step(self):
        if not hasattr(self, 'button_gid'):
            return

        def pressed(gid, qid):
            return self.data.qpos[qid] > 0.0025 and any((gid in (c.geom1, c.geom2) and any((self.model.geom_group[g] == 1 for g in (c.geom1, c.geom2))) for c in self.data.contact))
        start = pressed(self.button_gid, self.button_qid)
        stop = pressed(self.stop_button_gid, self.stop_button_qid)
        event = None
        if stop and self.belt_running:
            self.belt_running = False
            event = ('conveyor_stopped', self.stop_button_qid)
        elif start and (not self.start_pressed_last) and (not stop) and (not self.belt_running):
            self.belt_running = True
            event = ('conveyor_started', self.button_qid)
        self.start_pressed_last = start
        if event:
            self.events.append({'time': float(self.data.time), 'type': event[0], 'source': 'robot_contact', 'button_travel_m': float(self.data.qpos[event[1]])})
