"""Check main-model assets, seven parcels, stepping and derived homepage geometry."""
import hashlib, json, pathlib, sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import numpy as np
from hxsim.workstation import WorkstationSimulation
from hxsim.runtime import ROOT
s = WorkstationSimulation()
assert len(s.parcels) == 7
for parcel in s.parcels:
    assert s.model.body(parcel['id']).id > 0
    assert (ROOT/'scenes/labels'/f"{parcel['tracking_id']}.png").is_file()
assert s.model.geom('Conveyor_StartCap').id != s.model.geom('Conveyor_EStop').id
assert len(s.belt_aids) == 30
s.step(1000)
assert np.isfinite(s.data.qpos).all()
assert not np.any(s.data.warning.number)
# Explicit driver check, not a claim of robot-operated button pressing.
s.belt_running = True
s.step(100)
assert np.allclose(s.target[s.belt_aids], s.belt_speed)
s.belt_running = False
s.step()
assert np.allclose(s.target[s.belt_aids], 0)
preview = json.loads((ROOT/'scenes/g2_environment.json').read_text())
assert preview['model_sha256'] == hashlib.sha256((ROOT/'scenes/g2_operation.xml').read_bytes()).hexdigest()
assert len(preview['objects']) == s.model.ngeom
assert len(preview['initial']['pos']) == s.model.nbody
print(f'PASS: {len(s.parcels)} parcels, {s.model.ngeom} geoms, stable stepping, belt driver, homepage model SHA')
