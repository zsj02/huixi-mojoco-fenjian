from pathlib import Path
import json
import numpy as np
import mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
ROOT = Path(__file__).resolve().parents[1]

class G2Simulation:

    def __init__(self, model_path=None):
        self.model = mujoco.MjModel.from_xml_path(str(model_path or ROOT / 'scenes/g2_operation.xml'))
        self.data = mujoco.MjData(self.model)
        self.config = json.loads((ROOT / 'scenes/g2_configuration.json').read_text())
        self.frames = []
        self.aids = self.model.actuator_trnid[:, 0]
        self.qids = self.model.jnt_qposadr[self.aids]
        self.dids = self.model.jnt_dofadr[self.aids]
        self.target = self.data.qpos[self.qids].copy()
        self.steps = 0
        self.max_unexpected_penetration = 0.0
        self.max_limit_excess = 0.0
        self.limited = np.where(self.model.jnt_limited)[0]

    def ik(self, side, position, rotation=None, seed=None, collision_check=False, regularization=0.0):
        m = self.model
        d = mujoco.MjData(m)
        d.qpos[:] = self.data.qpos
        off = 21 if side == 'l' else 61
        ids = np.array([m.joint(f'idx{off + i}_arm_{side}_joint{i + 1}').id for i in range(7)])
        qi = m.jnt_qposadr[ids]
        sid = m.site(side + '_tcp').id
        bounds = m.jnt_range[ids]
        rot = np.diag([1, -1, -1]) if rotation is None else rotation

        def residual(q):
            d.qpos[qi] = q
            mujoco.mj_forward(m, d)
            return np.r_[d.site_xpos[sid] - position, 0.3 * Rotation.from_matrix(rot @ d.site_xmat[sid].reshape(3, 3).T).as_rotvec()]
        candidates = [np.clip(d.qpos[qi] if seed is None else seed, bounds[:, 0] + 0.001, bounds[:, 1] - 0.001)]
        candidates += [np.random.default_rng(i).uniform(bounds[:, 0] * 0.7, bounds[:, 1] * 0.7) for i in range(12)]
        anchor = candidates[0].copy()

        def objective(q):
            return np.r_[residual(q), regularization * (q - anchor)] if regularization else residual(q)
        for q in candidates:
            res = least_squares(objective, q, bounds=(bounds[:, 0] + 0.0001, bounds[:, 1] - 0.0001), max_nfev=300)
            if np.linalg.norm(residual(res.x)) < (0.0003 if regularization else 0.0001):
                if collision_check:
                    d.qpos[qi] = res.x
                    mujoco.mj_forward(m, d)
                    bad = [c for c in d.contact if c.dist < -0.001 and (m.geom_group[c.geom1] == 1 or m.geom_group[c.geom2] == 1) and (m.geom('test_parcel_geom').id not in [c.geom1, c.geom2])]
                    if bad:
                        continue
                return (ids, res.x)
        raise RuntimeError('IK failed ' + str(res.fun))

    def before_step(self):
        pass

    def after_step(self):
        pass

    def step(self, n=1, record=False):
        (m, d) = (self.model, self.data)
        for _ in range(n):
            self.before_step()
            mujoco.mj_forward(m, d)
            kp = m.actuator_gainprm[:, 0]
            d.ctrl[:] = self.target + d.qfrc_bias[self.dids] / kp
            mujoco.mj_step(m, d)
            self.steps += 1
            self.after_step()
            qq = d.qpos[m.jnt_qposadr[self.limited]]
            limits = m.jnt_range[self.limited]
            self.max_limit_excess = max(self.max_limit_excess, float(np.max(limits[:, 0] - qq)), float(np.max(qq - limits[:, 1])))
            for c in d.contact:
                if (m.geom_group[c.geom1] == 1 or m.geom_group[c.geom2] == 1) and m.geom('test_parcel_geom').id not in [c.geom1, c.geom2]:
                    self.max_unexpected_penetration = max(self.max_unexpected_penetration, float(-c.dist))
            if record and self.steps % 40 == 0:
                self.frames.append({'time': round(d.time, 4), 'qpos': d.qpos.tolist()})
            if not np.isfinite(d.qpos).all() or np.any(d.warning.number):
                raise RuntimeError('MuJoCo instability')
