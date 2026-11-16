"""Export initial geometry of the main model, without any task recording."""
import pathlib, sys, json, hashlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import mujoco
from hxsim.runtime import ROOT
from hxsim.workstation import WorkstationSimulation
s = WorkstationSimulation()
m, d = s.model, s.data
manifest = json.loads((ROOT/'scenes/workstation_manifest.json').read_text())
objects = []
for i in range(m.ngeom):
    typ = m.geom_type[i]
    o = {
        "name": m.geom(i).name,
        "body": int(m.geom_bodyid[i]),
        "is_robot": int(m.geom_group[i]) == 1,
        "pos": m.geom_pos[i].tolist(),
        "quat": m.geom_quat[i].tolist(),
        "color": m.geom_rgba[i].tolist(),
        "size": m.geom_size[i].tolist(),
    }
    if typ == mujoco.mjtGeom.mjGEOM_MESH:
        mid = m.geom_dataid[i]
        o.update(
            kind="mesh",
            vertices=m.mesh_vert[
                m.mesh_vertadr[mid] : m.mesh_vertadr[mid] + m.mesh_vertnum[mid]
            ]
            .flatten()
            .tolist(),
            faces=m.mesh_face[
                m.mesh_faceadr[mid] : m.mesh_faceadr[mid] + m.mesh_facenum[mid]
            ]
            .flatten()
            .tolist(),
        )
    elif typ == mujoco.mjtGeom.mjGEOM_BOX:
        o["kind"] = "box"
    elif typ == mujoco.mjtGeom.mjGEOM_CYLINDER:
        o["kind"] = "cylinder"
    elif typ == mujoco.mjtGeom.mjGEOM_SPHERE:
        o["kind"] = "sphere"
    else:
        raise ValueError("Unhandled geom " + str(typ))
    if o["name"].endswith("_label"):
        parcel = next(
            (p for p in manifest["parcels"] if o["name"] == p["id"] + "_label"), None
        )
        if parcel:
            o["texture"] = (
                ""
                + "labels/"
                + parcel["tracking_id"]
                + ".png"
            )
    objects.append(o)

data = {'model_sha256': hashlib.sha256((ROOT/'scenes/g2_operation.xml').read_bytes()).hexdigest(),
        'initial': {'pos': d.xpos.tolist(), 'quat': d.xquat.tolist()}, 'objects': objects}
(ROOT/'scenes/g2_environment.json').write_text(json.dumps(data, separators=(',', ':')))
print('Exported initial environment:', len(objects), 'geometries; no task trajectory')
