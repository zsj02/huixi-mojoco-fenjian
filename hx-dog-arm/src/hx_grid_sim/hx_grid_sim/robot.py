"""Original primitive proxy, deliberately independent of proprietary robot assets.

Fixed stance + four driven wheels + Cartesian lift/reach + rotating wrist/gripper.
This is a task-interface proxy, not an M20S kinematic or dynamic model.
"""
import xml.etree.ElementTree as E
from pathlib import Path
from math import pi


def child(p, element_name, text=None, **attrs):
    e = E.SubElement(p, element_name, attrs)
    if text is not None:
        e.text = str(text)
    return e


MOTION_PROFILES = {
    'legacy': dict(track_m=.6, wheel_velocity_rad_s=3., lateral_compliance=0.,
                   max_angular_velocity=.5, max_angular_acceleration=.6),
    # Finite lateral tyre compliance avoids a rigid four-wheel contact model
    # pivoting around one axle. Wheel spacing, friction and torque stay physical.
    'fast_turn': dict(track_m=.6, wheel_velocity_rad_s=6., lateral_compliance=1.,
                      max_angular_velocity=1.6, max_angular_acceleration=4.),
    'fast_patrol': dict(track_m=.6, wheel_velocity_rad_s=6., lateral_compliance=1.,
                       max_angular_velocity=1.6, max_angular_acceleration=4.,
                       max_linear_velocity=.8, max_linear_acceleration=.8),
}


def build_robot(output, ideal_odometry=False, motion_profile='legacy'):
    if motion_profile not in MOTION_PROFILES:
        raise ValueError(f'Unknown motion profile: {motion_profile}')
    motion = MOTION_PROFILES[motion_profile]
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    sdf = E.Element('sdf', version='1.9')
    m = child(sdf, 'model', name='wheel_legged_proxy')
    child(m, 'self_collide', 'false')
    urdf = E.Element('robot', name='wheel_legged_proxy')
    records = {}

    def make_link(name, xyz, size, mass=1, color='.12 .2 .25 1', wheel=False):
        link = child(m, 'link', name=name)
        child(link, 'pose', ' '.join(map(str, [*xyz, 0, 0, 0])))
        inertia = child(link, 'inertial')
        child(inertia, 'mass', mass)
        tensor = child(inertia, 'inertia')
        # Solid box/cylinder principal inertias, wheel cylinder axis is local Y.
        if wheel:
            r, length = size
            moment = (mass*(3*r*r+length*length)/12, mass*r*r/2,
                      mass*(3*r*r+length*length)/12)
        else:
            a,b,c = size
            moment = (mass*(b*b+c*c)/12, mass*(a*a+c*c)/12, mass*(a*a+b*b)/12)
        for n,v in zip(['ixx','iyy','izz'], moment):
            child(tensor,n,max(v, 1e-6))
        for n in ['ixy','ixz','iyz']:
            child(tensor,n,0)
        ulink = child(urdf, 'link', name=name)
        for typ in ['collision','visual']:
            elem=child(link,typ,name=name+'_'+typ)
            geom=child(elem,'geometry')
            uelem=child(ulink,typ)
            ugeom=child(uelem,'geometry')
            if wheel:
                child(elem,'pose', f'0 0 0 {pi/2} 0 0')
                g=child(geom,'cylinder'); child(g,'radius',size[0]); child(g,'length',size[1])
                child(uelem,'origin',rpy=f'{pi/2} 0 0')
                child(ugeom,'cylinder',radius=str(size[0]),length=str(size[1]))
            else:
                child(child(geom,'box'),'size',' '.join(map(str,size)))
                child(ugeom,'box',size=' '.join(map(str,size)))
            if wheel and typ=='collision' and motion['lateral_compliance']:
                # Collision cylinder's local Z is the wheel axle: first friction
                # direction is lateral; secondary direction is longitudinal.
                friction=child(child(child(elem,'surface'),'friction'),'ode')
                child(friction,'mu',.9);child(friction,'mu2',.9)
                child(friction,'fdir1','0 0 1')
            if typ=='visual':
                mat=child(elem,'material')
                child(mat,'ambient',color); child(mat,'diffuse',color)
                child(child(uelem,'material',name=name+'_color'),'color',rgba=color)
        records[name]=xyz
        return link

    def connect(name, typ, parent, target, axis='1 0 0', lo=0, hi=1, effort=100):
        j=child(m,'joint',name=name,type=typ)
        child(j,'parent',parent); child(j,'child',target)
        uj=child(urdf,'joint',name=name,type='continuous' if typ=='revolute' and lo==-1e16 else typ)
        child(uj,'parent',link=parent); child(uj,'child',link=target)
        diff=[records[target][i]-records[parent][i] for i in range(3)]
        child(uj,'origin',xyz=' '.join(map(str,diff)),rpy='0 0 0')
        if typ != 'fixed':
            a=child(j,'axis'); child(a,'xyz',axis)
            lim=child(a,'limit')
            wheel_velocity = motion['wheel_velocity_rad_s'] if name.endswith('_wheel_joint') else 3
            # Preserve byte-for-byte legacy assets, including integer formatting.
            if motion_profile=='legacy': wheel_velocity=3
            for n,v in [('lower',lo),('upper',hi),('effort',effort),('velocity',wheel_velocity)]: child(lim,n,v)
            child(child(a,'dynamics'),'damping',.3)
            child(uj,'axis',xyz=axis)
            child(uj,'limit',lower=str(lo),upper=str(hi),effort=str(effort),velocity=str(wheel_velocity))

    # Root is at floor height. No root collision: it is a frame with tiny inertia.
    root=child(m,'link',name='base_footprint')
    child(urdf,'link',name='base_footprint'); records['base_footprint']=(0,0,0)
    body=make_link('base_link',(0,0,.36),(.72,.42,.18),20)
    connect('body_mount','fixed','base_footprint','base_link')
    for x, label in [(.27,'front'),(-.27,'rear')]:
        for y,side in [(.30,'left'),(-.30,'right')]:
            n=f'{label}_{side}'
            make_link(n+'_leg',(x,y,.25),(.065,.06,.24),.5,color='.8 .65 .1 1')
            connect(n+'_stance','fixed','base_link',n+'_leg')
            make_link(n+'_wheel',(x,y,.14),(.14,.07),.8,color='.06 .06 .06 1',wheel=True)
            connect(n+'_wheel_joint','revolute',n+'_leg',n+'_wheel','0 1 0',-1e16,1e16,30)
    make_link('lift',(.1,0,.48),(.08,.08,.10),.6,color='.8 .65 .1 1')
    connect('lift_joint','prismatic','base_link','lift','0 0 1',0,1.1)
    make_link('lateral',(.1,0,.48),(.07,.07,.07),.2)
    connect('lateral_joint','prismatic','lift','lateral','0 1 0',-.2,.2)
    make_link('reach',(.28,0,.48),(.30,.055,.055),.4)
    connect('reach_joint','prismatic','lateral','reach','1 0 0',0,.60)
    make_link('pitch',(.44,0,.48),(.035,.05,.05),.15)
    connect('wrist_pitch_joint','revolute','reach','pitch','0 1 0',-pi/2,pi/2,4)
    make_link('wrist',(.44,0,.48),(.04,.055,.055),.2)
    connect('wrist_joint','revolute','pitch','wrist','1 0 0',-pi,pi,4)
    for side,sign in [('left',1),('right',-1)]:
        make_link(side+'_finger',(.50,sign*.018,.48),(.10,.012,.035),.05,
                  color='.85 .85 .8 1')
        connect(side+'_finger_joint','prismatic','wrist',side+'_finger',f'0 {sign} 0',0,.075,12)
    # Tall optical viewpoint sees both tiers. TF uses normal ROS optical axes.
    sensor_link=make_link('sensor_mount',(.15,0,.98),(.055,.06,.055),.1)
    connect('sensor_mount_joint','fixed','base_link','sensor_mount')
    records['camera_optical_frame']=records['sensor_mount']
    child(urdf,'link',name='camera_optical_frame')
    u=child(urdf,'joint',name='camera_optical_joint',type='fixed')
    child(u,'parent',link='sensor_mount'); child(u,'child',link='camera_optical_frame')
    child(u,'origin',xyz='0 0 0',rpy=f'{-pi/2} 0 {-pi/2}')
    camera=child(sensor_link,'sensor',name='rgbd_camera',type='rgbd_camera')
    child(camera,'topic','/camera'); child(camera,'update_rate',5)
    child(camera,'gz_frame_id','camera_optical_frame')
    cam=child(camera,'camera'); child(cam,'horizontal_fov',1.57)
    child(cam,'optical_frame_id','camera_optical_frame')
    image=child(cam,'image'); child(image,'width',640); child(image,'height',480); child(image,'format','R8G8B8')
    clip=child(cam,'clip'); child(clip,'near',.05); child(clip,'far',25)
    laser=child(sensor_link,'sensor',name='lidar',type='gpu_lidar')
    child(laser,'topic','/scan'); child(laser,'update_rate',10); child(laser,'gz_frame_id','sensor_mount')
    lidar=child(laser,'lidar'); horizontal=child(child(lidar,'scan'),'horizontal')
    for n,v in [('samples',720),('resolution',1),('min_angle',-pi),('max_angle',pi)]: child(horizontal,n,v)
    r=child(lidar,'range')
    for n,v in [('min',.08),('max',25),('resolution',.01)]: child(r,n,v)
    imu=child(body,'sensor',name='imu',type='imu')
    child(imu,'topic','/imu'); child(imu,'update_rate',100); child(imu,'gz_frame_id','base_link')
    dd=child(m,'plugin',filename='gz-sim-diff-drive-system',name='gz::sim::systems::DiffDrive')
    for label in ['front','rear']:
        child(dd,'left_joint',label+'_left_wheel_joint'); child(dd,'right_joint',label+'_right_wheel_joint')
    for n,v in [('wheel_separation',motion['track_m']),('wheel_radius',.14),('topic','/cmd_vel'),
                ('odom_topic','/wheel_odom' if ideal_odometry else '/odom'),
                ('tf_topic','/wheel_tf' if ideal_odometry else '/tf'),('frame_id','odom'),
                ('child_frame_id','base_footprint'),('odom_publish_frequency',30),
                ('max_linear_velocity',motion.get('max_linear_velocity',.3)),('max_angular_velocity',motion['max_angular_velocity']),
                ('max_linear_acceleration',motion.get('max_linear_acceleration',.4)),('max_angular_acceleration',motion['max_angular_acceleration'])]: child(dd,n,v)
    if motion_profile!='legacy':
        child(dd,'min_angular_velocity',-motion['max_angular_velocity'])
        child(dd,'min_angular_acceleration',-motion['max_angular_acceleration'])
        if motion_profile=='fast_patrol':
            child(dd,'min_linear_acceleration',-1.)
    if motion['lateral_compliance']:
        slip=child(m,'plugin',filename='gz-sim-wheel-slip-system',name='gz::sim::systems::WheelSlip')
        # SDF defaults an omitted root inertial mass to 1 kg.
        load=sum(float(l.findtext('inertial/mass','1')) for l in m.findall('link'))*9.81/4
        for label in ['front','rear']:
            for side in ['left','right']:
                wheel=child(slip,'wheel',link_name=f'{label}_{side}_wheel')
                child(wheel,'slip_compliance_lateral',motion['lateral_compliance'])
                child(wheel,'slip_compliance_longitudinal',0.)
                child(wheel,'wheel_normal_force',load)
                child(wheel,'wheel_radius',.14)
    if ideal_odometry:
        odom=child(m,'plugin',filename='gz-sim-odometry-publisher-system',name='gz::sim::systems::OdometryPublisher')
        for n,v in [('odom_frame','odom'),('robot_base_frame','base_footprint'),
                    ('odom_topic','/odom'),('tf_topic','/tf'),('dimensions',2),
                    ('odom_publish_frequency',30),('gaussian_noise',0)]: child(odom,n,v)
    js=child(m,'plugin',filename='gz-sim-joint-state-publisher-system',name='gz::sim::systems::JointStatePublisher')
    child(js,'topic','/joint_states')
    for name, initial in [('lift',.1),('lateral',0),('reach',0),('wrist_pitch',0),('wrist',0),('left_finger',.04),('right_finger',.04)]:
        p=child(m,'plugin',filename='gz-sim-joint-position-controller-system',name='gz::sim::systems::JointPositionController')
        angular=name in ('wrist_pitch','wrist')
        for n,v in [('joint_name',name+'_joint'),('topic','/arm/'+name+'/position'),
                    ('p_gain',30 if angular else (150 if name=='lift' else 80)),('d_gain',.1 if angular else 10),
                    ('i_gain',5 if name=='lift' else 0),('cmd_offset',16.1865 if name=='lift' else 0),('i_max',15),('i_min',-15),
                    ('cmd_max',4 if angular else (50 if name=='lift' else 12)),
                    ('cmd_min',-4 if angular else (-50 if name=='lift' else -12)),
                    ('initial_position',initial)]: child(p,n,v)
    for tree,filename in [(sdf,'robot.sdf'),(urdf,'robot.urdf')]:
        E.indent(tree)
        E.ElementTree(tree).write(output/filename,encoding='utf-8',xml_declaration=True)
    return output/'robot.sdf', output/'robot.urdf'
