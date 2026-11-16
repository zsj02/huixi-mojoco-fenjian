#!/usr/bin/env python3
"""Deterministic, dependency-free SDF / Nav2 map / manifest generation."""
import json
import math
from pathlib import Path
import random
import xml.etree.ElementTree as E

ROOT = Path(__file__).resolve().parents[1]
SCENES = {
    '01_grid_baseline': ('电网巡检基准走廊', 'baseline'),
    '02_outdoor_switchyard': ('日照碎石开关站', 'outdoor'),
    '03_utility_tunnel': ('阶坡遮挡管廊', 'tunnel'),
    '04_access_control_rooms': ('刷卡开门双机房', 'rooms'),
    '05_rotary_thermal_lab': ('旋钮测温扩展走廊', 'thermal'),
}
GRAY = '.52 .59 .63 1'
DARK = '.08 .12 .15 1'


def tag(parent, element_name, text=None, **attrs):
    el = E.SubElement(parent, element_name, attrs)
    if text is not None:
        el.text = str(text)
    return el


def pose(parent, xyzrpy):
    return tag(parent, 'pose', ' '.join(f'{v:.7g}' for v in xyzrpy))


def material(parent, color, emissive=False):
    mat = tag(parent, 'material')
    for n in ['ambient', 'diffuse'] + (['emissive'] if emissive else []):
        tag(mat, n, color)


def shape(parent, name, size, p=(0, 0, 0, 0, 0, 0), color=GRAY,
          collision=True, kind='box', emissive=False):
    # Principal inertia of the primary collision. Secondary trim is massless.
    if collision and not parent.findall('collision'):
        inertial = parent.find('inertial')
        mass = float(inertial.findtext('mass'))
        if kind == 'box':
            a,b,c = size
            moments = [mass*(b*b+c*c)/12, mass*(a*a+c*c)/12, mass*(a*a+b*b)/12]
        elif kind == 'sphere':
            moments = [2*mass*size[0]**2/5]*3
        else:
            r,h = size
            moments = [mass*(3*r*r+h*h)/12]*2+[mass*r*r/2]
        pose(inertial, p)
        for key,value in zip(['ixx','iyy','izz'],moments):
            inertial.find('inertia/'+key).text=str(value)
    for typ in (['collision', 'visual'] if collision else ['visual']):
        el = tag(parent, typ, name=name+'_collision' if typ=='collision' else name)
        pose(el, p)
        g = tag(tag(el, 'geometry'), kind)
        if kind == 'box':
            tag(g, 'size', ' '.join(map(str, size)))
        elif kind == 'sphere':
            tag(g, 'radius', size[0])
        else:
            tag(g, 'radius', size[0])
            tag(g, 'length', size[1])
        if typ == 'visual':
            material(el, color, emissive)
        else:
            surface = tag(el, 'surface')
            ode = tag(tag(surface, 'friction'), 'ode')
            tag(ode, 'mu', .9)
            tag(ode, 'mu2', .9)
    return parent


def link(model, name, p=(0, 0, 0, 0, 0, 0), mass=1.):
    el = tag(model, 'link', name=name)
    pose(el, p)
    inertia = tag(el, 'inertial')
    tag(inertia, 'mass', mass)
    tensor = tag(inertia, 'inertia')
    for n, v in [('ixx', .01*mass), ('iyy', .01*mass), ('izz', .01*mass),
                 ('ixy', 0), ('ixz', 0), ('iyz', 0)]:
        tag(tensor, n, v)
    return el


def joint(model, name, typ, parent, child, axis='1 0 0', limits=(0, .012),
          damping=1., spring=0., p=None):
    el = tag(model, 'joint', name=name, type=typ)
    if p is not None:
        pose(el, p)
    tag(el, 'parent', parent)
    tag(el, 'child', child)
    if typ != 'fixed':
        a = tag(el, 'axis')
        tag(a, 'xyz', axis)
        lim = tag(a, 'limit')
        tag(lim, 'lower', limits[0]); tag(lim, 'upper', limits[1])
        tag(lim, 'effort', 120); tag(lim, 'velocity', 2)
        dyn = tag(a, 'dynamics')
        tag(dyn, 'damping', damping)
        tag(dyn, 'spring_reference', 0)
        tag(dyn, 'spring_stiffness', spring)
    return el


def model(world, name, p=(0, 0, 0, 0, 0, 0), static=False):
    m = tag(world, 'model', name=name)
    pose(m, p)
    tag(m, 'static', str(static).lower())
    return m


def block(world, name, xyz, size, color=GRAY, rpy=(0, 0, 0)):
    m = model(world, name, (*xyz, *rpy), True)
    shape(link(m, 'body'), 'body', size, color=color)
    return m


DIGITS = {'0': 'abcdef', '1': 'bc', '2': 'abged', '3': 'abgcd',
          '4': 'fgbc', '5': 'afgcd', '6': 'afgecd', '7': 'abc',
          '8': 'abcdefg', '9': 'abfgcd', '-': 'g'}


def display(body, prefix, text, x, y, z, scale=.045, color='.1 1 .6 1'):
    """Seven-segment digits on local -X front, readable by RGB cameras."""
    span = len(text) * scale * 1.45
    for i, char in enumerate(text):
        cy = y + span/2 - (i+.5)*scale*1.45
        if char == '.':
            shape(body, f'{prefix}_{i}_dot', (.006, .009, .009),
                  (x, cy+scale*.5, z-scale, 0, 0, 0), color, False, emissive=True)
            continue
        for seg in DIGITS[char]:
            dy, dz, horizontal = {
                'a': (0, 1, True), 'b': (-.5, .5, False),
                'c': (-.5, -.5, False), 'd': (0, -1, True),
                'e': (.5, -.5, False), 'f': (.5, .5, False),
                'g': (0, 0, True)}[seg]
            size = (.006, scale if horizontal else .008,
                    .008 if horizontal else scale)
            shape(body, f'{prefix}_{i}_{seg}', size,
                  (x, cy+dy*scale, z+dz*scale, 0, 0, 0), color, False, emissive=True)


def cabinet(world, item, outdoor=False, thermal=False):
    m = model(world, item['id'], item['pose'])
    body = link(m, 'cabinet', mass=30)
    # Anchored housing, dynamic switch links. Local front points toward -X.
    joint(m, 'anchor', 'fixed', 'world', 'cabinet')
    shape(body, 'housing', (.42, .82, .78), color='.47 .55 .52 1' if outdoor else GRAY)
    shape(body, 'front_panel', (.015, .76, .72), (-.218, 0, 0, 0, 0, 0), '.22 .3 .34 1')
    shape(body, 'meter_back', (.012, .49, .17), (-.23, 0, .19, 0, 0, 0), DARK, False)
    display(body, 'kwh', f"{item['meter_kwh']:.1f}", -.24, 0, .19, .027)
    # Lamp and separate actuator. Button centered Y=-.16, lamp Y=+.22.
    color = '1 .02 .01 1' if item['initial_fault'] else '.02 1 .05 1'
    shape(body, 'indicator', (.042,), (-.25, .22, -.12, 0, 0, 0), color,
          False, 'sphere', True)
    if thermal:
        shape(body, 'temperature_back', (.012, .32, .11), (-.23, 0, .015, 0, 0, 0), DARK, False)
        display(body, 'celsius', f"{item['temperature_c']:.1f}", -.24, 0, .015,
                .020, '1 .62 .05 1')
    parent = 'cabinet'
    if item['actuator'] == 'rotate_then_press':
        knob = link(m, 'rotor', (-.25, -.16, -.12, 0, 0, 0), .15)
        shape(knob, 'grip', (.045, .14, .035), color='.9 .65 .08 1')
        joint(m, 'rotary_joint', 'revolute', 'cabinet', 'rotor', '1 0 0',
              (0, math.pi/2), damping=.25)
        parent = 'rotor'
    button = link(m, 'button', (-.28, -.16, -.12, 0, 0, 0), .08)
    shape(button, 'cap', (.025, .075, .075), color='.15 .32 .8 1')
    # Inward +X spring travel. Rotary gate is imposed by the interaction plugin.
    joint(m, 'press_joint', 'prismatic', parent, 'button', spring=180, damping=2)
    if outdoor:
        shape(body, 'rain_hood', (.55, .94, .05), (0, 0, .43, 0, .04, 0))
    if not outdoor:
        for iy in [-.30, .30]:
            shape(body, f'wall_mount_{iy}', (.32, .05, .05),
                  (.37, iy, 0, 0, 0, 0), DARK)
    return m


def door(world):
    m = model(world, 'access_door')
    frame = link(m, 'frame')
    joint(m, 'anchor', 'fixed', 'world', 'frame')
    for y in [-.9, .9]:
        shape(frame, f'post_{y}', (.22, .15, 2.4), (0, y, 1.2, 0, 0, 0))
    shape(frame, 'lintel', (.22, 1.95, .16), (0, 0, 2.4, 0, 0, 0))
    leaf = link(m, 'leaf', (0, -.8, 1.15, 0, 0, 0), 18)
    shape(leaf, 'door', (.08, 1.6, 2.28), (0, .8, 0, 0, 0, 0), '.16 .28 .39 1')
    # Initially locked. Opens into room B (positive X) when hinge turns negative.
    joint(m, 'hinge', 'revolute', 'frame', 'leaf', '0 0 1', (0, 0), damping=3)
    handle = link(m, 'handle', (-.09, .58, 1., 0, 0, 0), .25)
    shape(handle, 'lever', (.05, .22, .035), (0, -.08, 0, 0, 0, 0), '.85 .75 .3 1')
    joint(m, 'handle_joint', 'revolute', 'leaf', 'handle', '1 0 0',
          (-1.1, 0), damping=.3, spring=.5)
    shape(frame, 'reader', (.06, .22, .28), (-.17, -1.22, 1., 0, 0, 0), DARK)
    shape(frame, 'reader_led', (.015, .12, .025), (-.21, -1.22, 1.08, 0, 0, 0),
          '.1 .3 1 1', False, emissive=True)
    # Dynamic card, no command that bypasses physical swipe.
    table(world, 'card_pedestal', (-1.5, -1.35), .65, (.7, .7))
    card = model(world, 'authorized_card', (-1.5, -1.35, .684, 0, 0, 0))
    shape(link(card, 'card', mass=.025), 'card', (.086, .054, .008), color='.9 .9 .95 1')


def table(world, name, xy, height=.72, size=(1.2, .65)):
    m = model(world, name, (*xy, 0, 0, 0, 0), True)
    body = link(m, 'body')
    shape(body, 'top', (*size, .05), (0, 0, height, 0, 0, 0), '.55 .38 .23 1')
    for i, x in enumerate([-.4*size[0], .4*size[0]]):
        for j, y in enumerate([-.4*size[1], .4*size[1]]):
            shape(body, f'leg_{i}_{j}', (.05, .05, height),
                  (x, y, height/2, 0, 0, 0), DARK)
    return m


def make_scene(name, title, kind):
    length, width = (20., 3.2) if kind == 'tunnel' else (12., 4.8)
    data = {'schema_version': 1, 'id': name, 'name_zh': title, 'kind': kind,
            'size_m': [length, width], 'spawn': [-2., 0., .01, 0.],
            'robot': 'wheel_legged_proxy', 'cabinets': [],
            'evaluation': {'seed': 20, 'fault_interval_s': 0,
                           'truth_topic': '/evaluation/ground_truth',
                           'policy_may_read_truth': False},
            'capabilities_required': ['navigation', 'visual_inspection', 'press'],
            'patrol_poses': [[-length/2+2, 0, math.pi], [length/2-2, 0, 0]],
            'obstacles': []}
    if kind == 'rooms':
        data['capabilities_required'] += ['card_grasp', 'card_swipe', 'handle_rotate', 'door_push']
        data['door'] = {'reader_pose': [-.21, -1.22, 1.], 'card_model': 'authorized_card',
                        'id': 'HX-GRID-001', 'swipe_distance_m': .10, 'hold_s': .25,
                        'authorization_timeout_s': 15., 'handle_threshold_rad': -.65,
                        'passable_angle_rad': -1.25}
    if kind == 'tunnel':
        data['capabilities_required'] += ['step_locomotion', 'ramp_locomotion']
        data['terrain'] = {'step_height_m': .10, 'steps': 3, 'ramp_run_m': 2.5,
                           'ramp_rise_m': .30, 'proxy_step_capable': False}
    if kind == 'outdoor':
        data['capabilities_required'] += ['rough_terrain_locomotion']
        data['lighting'] = {'sun_direction': [.6, -.4, -.85], 'shadows': True}
    if kind == 'thermal':
        data['capabilities_required'] += ['rotate_then_press']
        data['future_capabilities'] = ['tool_grasp', 'infrared_measurement', 'temperature_report']
        data['thermometer'] = {'model': 'ir_thermometer', 'table_pose': [4., -1.65, .72],
                               'status': 'graspable_rigid_body; measurement_policy_not_implemented'}
    root = E.Element('sdf', version='1.9')
    w = tag(root, 'world', name=name)
    tag(w, 'gravity', '0 0 -9.81')
    physics = tag(w, 'physics', name='physics', type='ignored')
    tag(physics, 'max_step_size', .001); tag(physics, 'real_time_factor', 1)
    for filename, cls in [('physics', 'Physics'), ('user-commands', 'UserCommands'),
                          ('scene-broadcaster', 'SceneBroadcaster'), ('sensors', 'Sensors'),
                          ('imu', 'Imu')]:
        plugin = tag(w, 'plugin', filename=f'gz-sim-{filename}-system', name=f'gz::sim::systems::{cls}')
        if cls == 'Sensors':
            tag(plugin, 'render_engine', 'ogre2')
    scene = tag(w, 'scene')
    tag(scene, 'ambient', '.5 .5 .5 1' if kind != 'outdoor' else '.35 .35 .4 1')
    tag(scene, 'background', '.55 .7 .85 1' if kind == 'outdoor' else '.12 .15 .18 1')
    tag(scene, 'shadows', 'true')
    sun = tag(w, 'light', name='sun' if kind == 'outdoor' else 'ceiling_light', type='directional')
    pose(sun, (0, 0, 8, 0, 0, 0)); tag(sun, 'cast_shadows', 'true')
    tag(sun, 'diffuse', '1 .96 .88 1'); tag(sun, 'specular', '.3 .3 .3 1')
    tag(sun, 'direction', '.6 -.4 -.85' if kind == 'outdoor' else '.15 .1 -1')
    attenuation = tag(sun, 'attenuation')
    tag(attenuation, 'range', 100); tag(attenuation, 'constant', 1)
    block(w, 'floor', (0, 0, -.1), (length+.4, width+.4, .2),
          '.4 .38 .33 1' if kind == 'outdoor' else '.35 .39 .42 1')
    # 2D occupancy rectangles use the same source as SDF geometry.
    def wall(n, x, y, sx, sy, h=2.8, z=None, color=GRAY):
        block(w, n, (x, y, h/2 if z is None else z), (sx, sy, h), color)
        data['obstacles'].append([x-sx/2, y-sy/2, x+sx/2, y+sy/2])
    for side in [-1, 1]:
        wall(f'end_{side}', side*(length/2+.1), 0, .2, width+.4, .15 if kind=='outdoor' else 2.8)
        if kind != 'outdoor':
            wall(f'side_{side}', 0, side*(width/2+.1), length, .2)
        else:
            # Low boundary, no roof: sun casts shadows across exposed cabinets.
            wall(f'curb_{side}', 0, side*(width/2+.1), length, .15, .15)
    if kind != 'outdoor':
        block(w, 'roof', (0, 0, 2.9), (length+.4, width+.4, .15), '.55 .58 .6 1')
    if kind == 'outdoor':
        rng = random.Random(20)
        # Physical stones, not a flat gravel texture. Preserve a stable spawn patch.
        for i in range(360):
            x, y = rng.uniform(-5.7, 5.7), rng.uniform(-2.25, 2.25)
            if abs(x+2)<.65 and abs(y)<.65:
                continue
            h = rng.uniform(.012, .045)
            block(w, f'gravel_{i:03}', (x, y, h/2),
                  (rng.uniform(.04, .11), rng.uniform(.04, .10), h),
                  f'{rng.uniform(.3,.6):.3f} .38 .34 1', (0, 0, rng.random()*math.pi))
        # Narrow shade mast is represented in the map and physically casts shadows.
        wall('shade_mast', 1.5, 1.95, .18, .18, 3.5)
        block(w, 'shade_beam', (1.5, .8, 3.4), (.25, 2.5, .2), DARK)
    if kind == 'tunnel':
        # Full width rise: three 10 cm steps, plateau, then a descending ramp.
        for i in range(3):
            block(w, f'step_{i+1}', (-2.6+i*.4, 0, (i+1)*.05),
                  (.4, width, (i+1)*.1), '.5 .5 .48 1')
        block(w, 'raised_landing', (-.8, 0, .15), (1.6, width, .3))
        angle = math.atan2(.3, 2.5)
        block(w, 'descending_ramp', (1.25, 0, .12),
              (math.hypot(2.5,.3), width, .06), '.5 .52 .5 1', (0, angle, 0))
        for side in [-1, 1]:
            block(w, f'pipe_{side}', (0, side*1.35, 1.9), (18, .22, .22), '.18 .4 .46 1')
        wall('occlusion_baffle_left', -4., 1.08, .22, 1.04)
        wall('occlusion_baffle_right', 4.5, -1.08, .22, 1.04)
        data['spawn'] = [-7., 0., .01, 0.]
    if kind == 'rooms':
        wall('partition_south', 0, -1.65, .2, 1.5)
        wall('partition_north', 0, 1.65, .2, 1.5)
        door(w)
        # Closed door stays a live lidar obstacle, not a permanent static map wall.
        data['obstacles'].append([-1.85, -1.7, -1.15, -1.0])
    for end in [-1, 1]:
        # local -X front; west group is rotated pi to face into corridor.
        for row in range(2):
            for col in range(2):
                idx = len(data['cabinets'])+1
                x, y, z = end*(length/2-.52), (col-.5)*1.05, .65+row*.9
                yaw = math.pi if end == -1 else 0.
                item = {'id': f'cabinet_{idx:02}', 'pose': [x, y, z, 0, 0, yaw],
                        'room': ('A' if end == -1 else 'B') if kind == 'rooms' else None,
                        'row': row, 'column': col,
                        'actuator': 'rotate_then_press' if kind == 'thermal' and idx%2==0 else 'press',
                        'initial_fault': idx in (1, 4, 5, 8), 'meter_kwh': 1200.+idx*137.3,
                        'temperature_c': 29.+idx*2.5,
                        'press_travel_m': .012, 'activation_travel_m': .006,
                        'activation_hold_s': .12, 'rotation_required_rad': math.pi/2,
                        'service_pose': [end*(length/2-1.65), y-end*.16, 0 if end==1 else math.pi]}
                data['cabinets'].append(item)
                cabinet(w, item, kind=='outdoor', kind=='thermal')
                if row == 0 and kind == 'outdoor':
                    block(w, f'cabinet_stand_{end}_{col}', (x, y, .13), (.45, .86, .26), DARK)
                    block(w, f'cabinet_rack_{end}_{col}', (x+end*.26, y, 1.02), (.08, .86, 2.04), DARK)
                if row == 0:
                    data['obstacles'].append([x-.23, y-.43, x+.23, y+.43])
    if kind == 'thermal':
        table(w, 'tool_table', (4., -1.65))
        data['obstacles'].append([3.4, -1.975, 4.6, -1.325])
        gun = model(w, 'ir_thermometer', (4., -1.65, .78, math.pi/2, 0, 0))
        tool = link(gun, 'tool', mass=.25)
        shape(tool, 'barrel', (.17, .055, .07), color='1 .66 .05 1')
        shape(tool, 'grip', (.055, .05, .12), (-.04, 0, -.075, 0, -.2, 0), DARK)
        shape(tool, 'lens', (.008, .04, .04), (.09, 0, 0, 0, 0, 0), DARK)
    plugin = tag(w, 'plugin', filename='hx_grid_interactions', name='hx_grid::GridInteractions')
    tag(plugin, 'scene_id', name)
    tag(plugin, 'fault_interval_s', 0)
    for c in data['cabinets']:
        cfg = tag(plugin, 'cabinet')
        for key, val in [('name', c['id']), ('fault', int(c['initial_fault'])),
                         ('rotary', int(c['actuator']=='rotate_then_press')),
                         ('temperature', c['temperature_c']), ('meter', c['meter_kwh'])]:
            tag(cfg, key, val)
    tag(plugin, 'access_control', int(kind == 'rooms'))
    return root, data


def write_map(folder, data):
    resolution = .05
    length, width = data['size_m']
    ox, oy = -length/2-.5, -width/2-.5
    nx, ny = round((length+1)/resolution), round((width+1)/resolution)
    pixels = bytearray([254])*(nx*ny)
    for row in range(ny):
        y = oy+(ny-row-.5)*resolution
        for col in range(nx):
            x = ox+(col+.5)*resolution
            if any(a <= x <= c and b <= y <= d for a,b,c,d in data['obstacles']):
                pixels[row*nx+col] = 0
    (folder/'map.pgm').write_bytes(f'P5\n{nx} {ny}\n255\n'.encode()+pixels)
    (folder/'map.yaml').write_text(f'image: map.pgm\nmode: trinary\nresolution: {resolution}\norigin: [{ox}, {oy}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')


def write_preview(folder, data):
    length, width = data['size_m']
    def screen(x, y):
        return 70+(x+length/2)*45, 100+(width/2-y)*45
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="1100" height="440" viewBox="0 0 1100 440">',
           '<rect width="1100" height="440" fill="#10212c"/>',
           f'<text x="35" y="40" fill="#edf4f5" font-size="24">{data["id"]}</text>',
           '<text x="35" y="69" fill="#a8c5cb" font-size="15">Plan view / 8 cabinets / each end: 2 columns x 2 tiers</text>']
    for a,b,c,d in data['obstacles']:
        x,y = screen(a,d)
        svg.append(f'<rect x="{x}" y="{y}" width="{(c-a)*45}" height="{(d-b)*45}" fill="#617d8b"/>')
    for c in data['cabinets']:
        x,y = screen(c['pose'][0], c['pose'][1])
        if c['row']==0:
            svg.append(f'<text x="{x-15}" y="{y-24}" fill="#ffcd62" font-size="13">{c["id"][-2:]}/{int(c["id"][-2:])+2:02}</text>')
    x,y=screen(*data['spawn'][:2])
    svg.append(f'<circle cx="{x}" cy="{y}" r="12" fill="#55dfc1"/>')
    if data['kind']=='rooms':
        x,y=screen(0,.8)
        svg.append(f'<path d="M{x},{y} v72" stroke="#ffab66" stroke-width="5"/>')
    if data['kind']=='tunnel':
        for sx in [-2.8,-2.4,-2.]:
            x,y=screen(sx,width/2)
            svg.append(f'<path d="M{x},{y} v{width*45}" stroke="#ffab66" stroke-width="5"/>')
        x,y=screen(0,width/2)
        svg.append(f'<rect x="{x}" y="{y}" width="112.5" height="{width*45}" fill="#e4b86b" opacity=".4"/>')
    svg += ['<text x="35" y="405" fill="#a8c5cb" font-size="15">Meters: kWh | red: service required | green: normal | Z tiers: 0.65 / 1.55 m</text>', '</svg>']
    (folder/'preview.svg').write_text('\n'.join(svg))


def scene_readme(data):
    """Layout documentation uses the same coordinates as the generated scene."""
    name, kind = data['id'], data['kind']
    length, width = data['size_m']
    notes = {'baseline': '室内过道，两端各 2×2 电箱，共 8 个。',
             'outdoor': '露天防雨柜、实体碎石、定向太阳与遮阴梁。',
             'tunnel': '20 m 狭长管廊、3 级 10 cm 台阶、30 cm 高平台、2.5 m 坡道、管道与遮挡板。代理机器人没有爬阶步态。',
             'rooms': '两室各 4 柜。动态门、实体门禁卡、读卡区、可旋转把手、受锁约束的门铰链。',
             'thermal': '与 baseline 相同的柜体布局，4 个普通按钮、4 个旋转 90° 后按压的按钮、8 个温度显示、工具桌与动态温度枪。测温策略留待后续。'}[kind]
    boundary = '露天边界低沿高 0.15 m，无屋顶' if kind == 'outdoor' else '墙高 2.8 m；屋顶下沿 2.825 m'
    text = f'''# {data['name_zh']}

{notes}

启动：`python3 tools/view_environment.py {name}`；无界面：`python3 tools/view_environment.py {name} --headless`。

`world.sdf` 为独立世界，`scene.json` 为布局及评测元数据，`map.yaml` / `map.pgm` 为 Nav2 地图，`preview.svg` 为俯视示意。机器人在启动时单独生成。

## 尺寸与坐标

单位为米。原点在场地几何中心的地面，X 沿长度方向，Y 沿宽度方向，Z 向上。下表电箱坐标是本体几何中心，两端柜门均朝场地内部；不是按钮坐标或机器人导航目标。尺寸和灯色来自本目录的场景文件，不提供给策略作为预知答案。

| 项目 | 尺寸或位置 |
| --- | --- |
| {'名义场地长 × 宽' if kind=='outdoor' else '内部场地长 × 宽'} | {length:g} × {width:g} m |
| 墙体或露天边界 | {boundary} |
| 地板模型长 × 宽 × 厚 | {length+.4:g} × {width+.4:g} × 0.2 m，包含墙体外围，不是室内净尺寸 |
| 电箱本体宽 Y × 高 Z × 深 X | 0.82 × 0.78 × 0.42 m，不含按钮、安装支架及防雨罩 |
| 同排两柜中心间距 / 本体净间隙 | 1.05 / 0.23 m |
| 上下两排中心高度 / 间距 | 0.65、1.55 / 0.90 m |
| 下排本体底部 / 上排本体顶部高度 | 0.26 / 1.94 m |
| 两端前面板之间距离 | 约 {length-1.491:g} m |
| 导航地图单列电箱占地矩形 | 深 0.46 × 宽 0.86 m，四边各比本体多 0.02 m；不是柜体尺寸 |
| 按钮端面宽 × 高 | 0.075 × 0.075 m；帽厚 0.025 m |
| 下排 / 上排按钮与灯中心高度 | 0.53 / 1.43 m |
| 指示灯球体直径 | 0.084 m |

| 电箱 | X | Y | Z | 初始灯色 | 操作 |
| --- | ---: | ---: | ---: | --- | --- |
'''
    for c in data['cabinets']:
        x,y,z = c['pose'][:3]
        action = '旋转 90° 后按压' if c['actuator']=='rotate_then_press' else '直接按压'
        text += f"| {c['id']} | {x:+.3f} | {y:+.3f} | {z:.2f} | {'红' if c['initial_fault'] else '绿'} | {action} |\n"
    spawn = data['spawn']
    text += f'''
默认出生位置为 ({spawn[0]:g}, {spawn[1]:g}, {spawn[2]:g})，yaw={spawn[3]:g} rad。本归档启动器使用这里登记的默认出生位置。其他场景使用各自默认出生位置；场地几何中心不一定是适合出生的地面。

按钮及灯在电箱本地坐标分别为 (-0.28, -0.16, -0.12)、(-0.25, +0.22, -0.12)。本地前方为 -X；负 X 端电箱绕 Z 轴旋转 π，因此其横向偏移符号也反转。端面最外侧还需考虑帽厚的一半。

## 触发与场景差异

初始 4 红 4 绿。按压总行程 0.012 m，至少推进 0.006 m 并保持 0.12 s 才触发变绿。'''
    if kind == 'thermal':
        text += '偶数编号电箱必须先旋转 90° 解锁，其余电箱直接按压。\n\n工具桌中心 XY=(4, -1.65)，桌面中心高度 0.72 m、尺寸 1.2 × 0.65 m、厚 0.05 m；温度枪为可抓取动态物体。8 柜温度为场景显示值，测温与上报策略尚未实现。\n'
    else:
        text += '本环境全部为普通按钮，没有旋转解锁要求。\n'
    if kind == 'outdoor':
        text += '\n名义宽度为4.8 m；两侧低沿中心 Y=±2.5 m、厚0.15 m，实际低沿内侧之间净宽4.85 m。防雨罩尺寸为 0.55（深）× 0.94（宽）× 0.05（厚）m，中心相对柜体 Z=+0.43 m；碎石厚度约 0.012–0.045 m。定向太阳与遮阴结构产生阴影；上述柜高以统一世界地面 Z=0 为基准，并非相对每块碎石的高度。\n'
    elif kind == 'tunnel':
        text += '\n管廊长宽为 20 × 3.2 m，电箱 X=±9.48 m，与12米走廊不同。3级台阶每级高0.10 m、进深0.40 m，平台累计高0.30 m；坡道水平长度2.50 m、落差0.30 m。台阶、坡道、管道和遮挡板均为物理几何，当前固定轮足代理不具备爬阶步态。\n'
    elif kind == 'rooms':
        text += '\n隔墙位于 X=0，将12 × 4.8 m的总场地分为两室；每侧名义长度6 m，中央墙厚0.2 m，隔墙表面至端墙之间净长5.9 m。门叶尺寸0.08（厚）× 1.60（宽）× 2.28（高）m；读卡器中心(-0.17, -1.22, 1.00)，把手关节参考位置(-0.09, 0.58, 1.00)。门初始锁定，需实体卡刷卡、旋转把手并推门；不得将门后区域视为已经检查。几何中心(0, 0)位于门洞，默认出生点在A室。\n'
    return text+'\n详细控制接口与运行限制见根目录文档。几何信息更新后应重新生成本文件，冻结实验源码副本不随主线文档改写。\n'


def main():
    for name, (title, kind) in SCENES.items():
        folder = ROOT/'environments'/name
        folder.mkdir(parents=True, exist_ok=True)
        root, data = make_scene(name, title, kind)
        E.indent(root)
        E.ElementTree(root).write(folder/'world.sdf', encoding='utf-8', xml_declaration=True)
        (folder/'scene.json').write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n',encoding='utf-8')
        write_map(folder, data)
        write_preview(folder, data)
        (folder/'README.md').write_text(scene_readme(data),encoding='utf-8')
        print(name)

if __name__ == '__main__':
    main()
