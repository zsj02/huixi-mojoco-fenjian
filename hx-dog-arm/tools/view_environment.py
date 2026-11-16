"""Launch a canonical world with its proxy robot; no agent or ROS required."""
import argparse, json, os, pathlib, shutil, subprocess, tempfile
import xml.etree.ElementTree as E
ROOT = pathlib.Path(__file__).resolve().parents[1]
p = argparse.ArgumentParser()
p.add_argument('scene', choices=sorted(x.name for x in (ROOT/'environments').iterdir() if x.is_dir()))
p.add_argument('--headless', action='store_true')
p.add_argument('--write-world', type=pathlib.Path, help='Assemble world only; do not launch Gazebo')
a = p.parse_args()
folder = ROOT/'environments'/a.scene
tree = E.parse(folder/'world.sdf')
robot = E.parse(ROOT/'robots/wheel_legged_proxy/robot.sdf').getroot().find('model')
x, y, z, yaw = json.loads((folder/'scene.json').read_text())['spawn']
pose = robot.find('pose')
if pose is None: pose = E.SubElement(robot, 'pose')
pose.text = f'{x} {y} {z} 0 0 {yaw}'
tree.getroot().find('world').append(robot)
E.indent(tree)
if a.write_world:
    tree.write(a.write_world, encoding='utf-8', xml_declaration=True)
else:
    if not shutil.which('gz'): p.error('Install Gazebo Harmonic (gz-sim8) first')
    if not list((ROOT/'build').glob('*hx_grid_interactions*')): p.error('Build the interaction plugin first: cmake -S . -B build && cmake --build build')
    env = os.environ.copy()
    env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = str(ROOT/'build') + os.pathsep + env.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    with tempfile.TemporaryDirectory(prefix='hx-world-') as tmp:
        path = pathlib.Path(tmp)/'world.sdf'
        tree.write(path, encoding='utf-8', xml_declaration=True)
        cmd = ['gz', 'sim', '-r'] + (['-s', '--headless-rendering'] if a.headless else []) + [str(path)]
        raise SystemExit(subprocess.call(cmd, env=env))
