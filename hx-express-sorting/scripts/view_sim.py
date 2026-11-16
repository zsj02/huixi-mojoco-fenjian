"""Native viewer for the one shared model; macOS requires mjpython."""
import pathlib, sys, time
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import mujoco.viewer
from hxsim.workstation import WorkstationSimulation
s = WorkstationSimulation()
with mujoco.viewer.launch_passive(s.model, s.data) as viewer:
    while viewer.is_running():
        start = time.monotonic()
        with viewer.lock():
            s.step(10)
        viewer.sync()
        time.sleep(max(0, 10 * s.model.opt.timestep - (time.monotonic() - start)))
