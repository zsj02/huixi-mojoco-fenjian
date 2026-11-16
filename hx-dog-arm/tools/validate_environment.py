"""Offline environment checks; does not replace gz sdf or runtime validation."""
import pathlib, subprocess, sys, tempfile
import xml.etree.ElementTree as E
ROOT = pathlib.Path(__file__).resolve().parents[1]
subprocess.run([sys.executable, '-m', 'unittest', 'discover', '-s', 'tests', '-v'], cwd=ROOT, check=True)
with tempfile.TemporaryDirectory() as tmp:
    for scene in sorted((ROOT/'environments').iterdir()):
        if not scene.is_dir(): continue
        out = pathlib.Path(tmp)/(scene.name+'.sdf')
        subprocess.run([sys.executable, str(ROOT/'tools/view_environment.py'), scene.name, '--write-world', str(out)], check=True)
        tree = E.parse(out)
        robot = E.parse(ROOT/'robots/wheel_legged_proxy/robot.sdf').find('model')
        assert tree.find(f"world/model[@name='{robot.get('name')}']") is not None
print('PASS: five worlds and robot assembly; Gazebo runtime not exercised')
