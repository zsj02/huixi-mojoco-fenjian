import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as E
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/hx_grid_sim'))
from hx_grid_sim.robot import build_robot
spec = importlib.util.spec_from_file_location('generator', ROOT / 'tools/generate_environments.py')
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)

class Environments(unittest.TestCase):

    def setUp(self):
        self.scenes = [(p, json.loads((p / 'scene.json').read_text()), E.parse(p / 'world.sdf')) for p in sorted((ROOT / 'environments').iterdir()) if p.is_dir()]

    def test_five_complete_independent_scenes(self):
        self.assertEqual(len(self.scenes), 5)
        for (p, d, t) in self.scenes:
            for name in ['world.sdf', 'scene.json', 'map.pgm', 'map.yaml', 'preview.svg', 'README.md']:
                self.assertTrue((p / name).is_file(), f'{p}/{name}')
            self.assertEqual(t.find('world').get('name'), p.name)
            self.assertFalse(t.findall('.//include'))
            self.assertEqual(len(d['cabinets']), 8)
            self.assertEqual(len(t.findall(".//joint[@name='press_joint']")), 8)
            self.assertEqual(sum((c['initial_fault'] for c in d['cabinets'])), 4)
            self.assertEqual(len({c['meter_kwh'] for c in d['cabinets']}), 8)
            for end in [-1, 1]:
                group = [c for c in d['cabinets'] if c['pose'][0] * end > 0]
                self.assertEqual(len(group), 4)
                self.assertEqual({(c['row'], c['column']) for c in group}, {(0, 0), (0, 1), (1, 0), (1, 1)})

    def test_generation_is_deterministic_and_checked_in(self):
        for (p, d, tree) in self.scenes:
            (root, expected) = generator.make_scene(p.name, *generator.SCENES[p.name])
            self.assertEqual(d, expected)
            E.indent(root)
            with tempfile.TemporaryDirectory() as tmp:
                target = Path(tmp)
                E.ElementTree(root).write(target / 'world.sdf', encoding='utf-8', xml_declaration=True)
                generator.write_map(target, expected)
                generator.write_preview(target, expected)
                for file in ['world.sdf', 'map.pgm', 'map.yaml', 'preview.svg']:
                    self.assertEqual((p / file).read_bytes(), (target / file).read_bytes(), f'stale generated {p / file}')

    def test_unique_names_and_valid_physics(self):
        for (p, d, tree) in self.scenes:
            models = tree.findall('.//model')
            self.assertEqual(len({m.get('name') for m in models}), len(models))
            for m in models:
                links = {l.get('name') for l in m.findall('link')}
                names = [j.get('name') for j in m.findall('joint')]
                self.assertEqual(len(names), len(set(names)))
                for j in m.findall('joint'):
                    self.assertIn(j.findtext('child'), links)
                    self.assertIn(j.findtext('parent'), links | {'world'})
                    lo = j.findtext('axis/limit/lower')
                    hi = j.findtext('axis/limit/upper')
                    if lo is not None:
                        self.assertLessEqual(float(lo), float(hi))
                for l in m.findall('link'):
                    mass = l.findtext('inertial/mass')
                    self.assertGreater(float(mass), 0)
                    moments = [float(l.findtext('inertial/inertia/' + a)) for a in ['ixx', 'iyy', 'izz']]
                    self.assertTrue(all((v > 0 for v in moments)))
                    self.assertLessEqual(max(moments), sum(moments) - max(moments) + 1e-09)
                    ns = [e.get('name') for kind in ['collision', 'visual'] for e in l.findall(kind)]
                    self.assertEqual(len(ns), len(set(ns)))
            for box in tree.findall('.//geometry/box/size'):
                self.assertTrue(all((float(v) > 0 for v in box.text.split())))
            for vector in tree.findall('.//pose'):
                values = [float(v) for v in vector.text.split()]
                self.assertEqual(len(values), 6)
                self.assertTrue(all((math.isfinite(v) for v in values)))

    def test_maps_match_obstacles_and_free_spawn_goals(self):
        for (p, d, t) in self.scenes:
            (_, size, maximum, pixels) = (p / 'map.pgm').read_bytes().split(b'\n', 3)
            (nx, ny) = map(int, size.split())
            self.assertEqual(maximum, b'255')
            self.assertEqual(len(pixels), nx * ny)
            (length, width) = d['size_m']
            (ox, oy) = (-length / 2 - 0.5, -width / 2 - 0.5)

            def sample(x, y):
                col = int((x - ox) / 0.05)
                row = ny - 1 - int((y - oy) / 0.05)
                return pixels[row * nx + col]
            poses = [[d['spawn'][0], d['spawn'][1], d['spawn'][3]], *d['patrol_poses'], *[c['service_pose'] for c in d['cabinets']]]
            for pose in poses:
                for (dx, dy) in [(0, 0), (0.57, 0.35), (0.57, -0.35), (-0.4, 0.35), (-0.4, -0.35)]:
                    gx = pose[0] + dx * math.cos(pose[2]) - dy * math.sin(pose[2])
                    gy = pose[1] + dx * math.sin(pose[2]) + dy * math.cos(pose[2])
                    self.assertEqual(sample(gx, gy), 254, (p, pose, dx, dy))
            for row in range(0, ny, 3):
                for col in range(0, nx, 3):
                    x = ox + (col + 0.5) * 0.05
                    y = oy + (ny - row - 0.5) * 0.05
                    occupied = any((a <= x <= c and b <= y <= e for (a, b, c, e) in d['obstacles']))
                    self.assertEqual(pixels[row * nx + col], 0 if occupied else 254)

    def test_thermal_is_baseline_layout_plus_four_rotary_and_tool(self):
        base = self.scenes[0][1]
        thermal = self.scenes[4][1]
        tree = self.scenes[4][2]
        for (a, b) in zip(base['cabinets'], thermal['cabinets']):
            for key in ['pose', 'row', 'column', 'service_pose', 'initial_fault', 'meter_kwh']:
                self.assertEqual(a[key], b[key])
        self.assertEqual(sum((c['actuator'] == 'rotate_then_press' for c in thermal['cabinets'])), 4)
        self.assertEqual(len(tree.findall(".//joint[@name='rotary_joint']")), 4)
        self.assertEqual(len({c['temperature_c'] for c in thermal['cabinets']}), 8)
        tool = tree.find(".//model[@name='ir_thermometer']")
        self.assertEqual(tool.findtext('static'), 'false')
        self.assertTrue(tree.findall(".//visual[@name='celsius_0_a']"))

    def test_outdoor_has_sun_real_gravel_and_no_roof(self):
        tree = self.scenes[1][2]
        self.assertIsNotNone(tree.find(".//light[@name='sun']"))
        self.assertIsNone(tree.find(".//model[@name='roof']"))
        rocks = [m for m in tree.findall('.//model') if m.get('name').startswith('gravel_')]
        self.assertGreater(len(rocks), 300)
        for m in rocks:
            self.assertIsNotNone(m.find('.//collision'))
        self.assertEqual(tree.findtext('.//scene/shadows'), 'true')

    def test_tunnel_steps_ramp_and_occlusion_are_physical(self):
        tree = self.scenes[2][2]
        data = self.scenes[2][1]
        self.assertFalse(data['terrain']['proxy_step_capable'])
        for n in ['step_1', 'step_2', 'step_3', 'raised_landing', 'descending_ramp', 'occlusion_baffle_left', 'occlusion_baffle_right']:
            self.assertIsNotNone(tree.find(f".//model[@name='{n}']//collision"))
        for i in range(1, 4):
            size = tree.findtext(f".//model[@name='step_{i}']//box/size")
            self.assertAlmostEqual(float(size.split()[2]), i * 0.1)
        p = tree.findtext(".//model[@name='descending_ramp']/pose")
        self.assertGreater(float(p.split()[4]), 0)

    def test_door_is_locked_hinged_and_card_is_dynamic(self):
        tree = self.scenes[3][2]
        data = self.scenes[3][1]
        self.assertEqual(sum((c['room'] == 'A' for c in data['cabinets'])), 4)
        self.assertEqual(sum((c['room'] == 'B' for c in data['cabinets'])), 4)
        hinge = tree.find(".//joint[@name='hinge']")
        self.assertEqual(hinge.get('type'), 'revolute')
        self.assertEqual(float(hinge.findtext('axis/limit/upper')), 0)
        self.assertEqual(float(hinge.findtext('axis/limit/lower')), 0)
        self.assertEqual(tree.findtext(".//model[@name='authorized_card']/static"), 'false')
        self.assertIsNotNone(tree.find(".//joint[@name='handle_joint']"))
        self.assertIsNotNone(tree.find(".//collision[@name='reader_collision']"))

class RobotAndVision(unittest.TestCase):

    def test_robot_reproducible_and_matching_tf_joint_tree(self):
        with tempfile.TemporaryDirectory() as tmp:
            (sdf, urdf) = build_robot(tmp)
            s = E.parse(sdf)
            u = E.parse(urdf)
            self.assertEqual(sdf.read_bytes(), (ROOT / 'robots/wheel_legged_proxy/robot.sdf').read_bytes())
            self.assertEqual(urdf.read_bytes(), (ROOT / 'robots/wheel_legged_proxy/robot.urdf').read_bytes())
            self.assertEqual({j.get('name') for j in s.findall('.//joint')}, {j.get('name') for j in u.findall('joint')} - {'camera_optical_joint'})
            self.assertEqual(len(s.findall(".//joint[@type='fixed']")), 6)
            self.assertEqual(len(s.findall(".//plugin[@name='gz::sim::systems::DiffDrive']/left_joint")), 2)
            self.assertNotIn(b'use_velocity_commands', sdf.read_bytes())
            self.assertEqual(u.find(".//joint[@name='camera_optical_joint']/child").get('link'), 'camera_optical_frame')
if __name__ == '__main__':
    unittest.main()
