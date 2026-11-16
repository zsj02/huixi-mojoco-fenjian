"""Compile vendor G2 CRS + SwiftPicker into an executable MuJoCo workcell."""

import pathlib, xml.etree.ElementTree as E, json, math, tempfile, sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from hxsim.workcell_assets import configure_workcell
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation

R = pathlib.Path(__file__).resolve().parents[1]
D = R / "assets/g2_description"
OUT = R / "scenes"


def sub(parent, tag, **kw):
    return E.SubElement(parent, tag, {k: str(v) for k, v in kw.items()})


def fmt(x):
    return " ".join(f"{float(v):.10g}" for v in x)


def quat(mat):
    q = Rotation.from_matrix(mat).as_quat()
    return fmt(q[[3, 0, 1, 2]])


def main():
    root = E.parse(D / "g2_crs_swiftpicker.urdf").getroot()
    repairs = []
    for l in root.findall("link"):
        inertia = l.find("inertial/inertia")
        if inertia is not None:
            v = np.array([float(inertia.get(k)) for k in ["ixx", "iyy", "izz"]])
            assert all(
                abs(float(inertia.get(k, "0"))) < 1e-12 for k in ["ixy", "ixz", "iyz"]
            )
            k = int(np.argmax(v))
            excess = v[k] - v.sum() / 2
            if excess > 0:
                old = v.copy()
                v[k] -= excess + 1e-10
                v[np.arange(3) != k] += (excess + 1e-10) / 2
                for key, val in zip(["ixx", "iyy", "izz"], v):
                    inertia.set(key, str(val))
                repairs.append(
                    {
                        "link": l.get("name"),
                        "original": old.tolist(),
                        "corrected": v.tolist(),
                        "reason": "Vendor principal inertia violates triangle inequality; trace-preserving projection",
                    }
                )
        for v in l.findall("visual"):
            l.remove(v)
        for m in l.findall(".//mesh"):
            m.set(
                "filename",
                str(D / m.get("filename").split("package://genie_sim_robot_model/")[1]),
            )
    ext = sub(root, "mujoco")
    sub(ext, "compiler", strippath="false", fusestatic="false", discardvisual="true")
    model = mujoco.MjModel.from_xml_string(E.tostring(root, encoding="unicode"))
    with tempfile.TemporaryDirectory() as td:
        path = pathlib.Path(td) / "import.xml"
        mujoco.mj_saveLastXML(str(path), model)
        mj = E.parse(path).getroot()
    mj.set("model", "HX G2 CRS SwiftPicker operation cell")
    comp = mj.find("compiler")
    comp.set("angle", "radian")
    comp.set("meshdir", "../assets/g2_description")
    for mesh in mj.findall("asset/mesh"):
        mesh.set("file", str(pathlib.Path(mesh.get("file")).relative_to(D)))
    opt = mj.find("option")
    if opt is None:
        opt = sub(mj, "option")
    opt.attrib.update(
        timestep="0.001",
        integrator="implicitfast",
        solver="Newton",
        iterations="100",
        noslip_iterations="3",
        cone="elliptic",
        gravity="0 0 -9.81",
    )
    sub(mj, "size", njmax="4000", nconmax="1000")
    default = mj.find("default")
    if default is None:
        default = sub(mj, "default")
    sub(
        default,
        "geom",
        friction="0.8 0.01 0.001",
        condim="4",
        solref="0.008 1",
        solimp="0.95 0.99 0.001",
    )
    sub(
        default,
        "joint",
        damping="1",
        armature="0.01",
        solreflimit="0.004 1",
        margin="0.001",
    )
    world = mj.find("worldbody")
    wrapper = E.Element(
        "body",
        name="g2_mount",
        pos="-0.6 -1 0.046",
        quat=fmt([math.sqrt(0.5), 0, 0, math.sqrt(0.5)]),
    )
    for child in list(world):
        world.remove(child)
        wrapper.append(child)
    world.append(wrapper)
    sub(wrapper, "freejoint", name="g2_mobile_base")
    eq = sub(mj, "equality")
    act = sub(mj, "actuator")
    contacts = sub(mj, "contact")
    joints = {
        j.get("name"): j for j in root.findall("joint") if j.get("type") != "fixed"
    }
    configuration = {
        "engine": "MuJoCo " + mujoco.__version__,
        "variant": "G2 CRS + dual SwiftPicker",
        "base": "free rigid base driven by the original four steering/wheel pairs",
        "source_commit": json.loads((D / "provenance.json").read_text())["commit"],
        "inertia_repairs": repairs,
        "actuators": {},
        "assumptions": [
            "Mobile base uses vendor wheel geometry and torque limits; wheel-ground friction is simulation tuning, not hardware calibration",
            "Position-controller gains and contact friction are simulation tuning, not calibrated hardware",
            "Three NoSlip solver iterations suppress soft-contact friction drift; no parcel welds, pose overrides or added grasp forces",
            "SwiftPicker master torque capped at 18 Nm per upstream hardware-cap comment",
            "Internal collision pairs within each gripper excluded; gripper vs robot/environment remains enabled",
        ],
    }
    for j in wrapper.iter("joint"):
        src = joints[j.get("name")]
        name = j.get("name")
        mim = src.find("mimic")
        dyn = src.find("dynamics")
        if dyn is not None:
            j.set("damping", dyn.get("damping", "1"))
            j.set("frictionloss", dyn.get("friction", "0"))
        if mim is not None:
            sub(
                eq,
                "joint",
                name="mimic_" + name,
                joint1=name,
                joint2=mim.get("joint"),
                polycoef=f"{mim.get('offset','0')} {mim.get('multiplier','1')} 0 0 0",
                solref="0.003 1",
            )
            continue
        limit = src.find("limit")
        effort = float(limit.get("effort"))
        assert effort > 0, name
        kp, kv = (500, 40) if "arm_" in name else (2000, 100)
        if "gripper" in name:
            kp, kv, effort = 150, 4, min(effort, 18)
        if "chassis" in name:
            kp, kv = 100, 5
        attrs = dict(
            name=name + "_position",
            joint=name,
            kp=kp,
            kv=kv,
            forcerange=fmt([-effort, effort]),
            forcelimited="true",
        )
        if src.get("type") != "continuous":
            attrs.update(
                ctrlrange=f"{limit.get('lower')} {limit.get('upper')}",
                ctrllimited="true",
            )
        sub(act, "position", **attrs)
        configuration["actuators"][name] = {
            "kp": kp,
            "kv": kv,
            "effort_Nm": effort,
            "velocity_limit_rad_s": float(limit.get("velocity")),
        }
    # Geometry coloring and explicit internal-hand exclusions (closed linkage members overlap by design).
    for b in wrapper.iter("body"):
        name = b.get("name", "")
        for g in b.findall("geom"):
            g.set(
                "rgba", "0.78 0.82 0.84 1" if "arm" not in name else "0.25 0.3 0.33 1"
            )
            g.set("group", "1")
    body_names = {b.get("name") for b in wrapper.iter("body")}
    for pair in E.parse(D / "genie.srdf.xacro").getroot().findall("disable_collisions"):
        a, b = pair.get("link1"), pair.get("link2")
        if pair.get("reason") == "Adjacent" and a in body_names and b in body_names:
            sub(contacts, "exclude", body1=a, body2=b)
    for side in ["l", "r"]:
        hand = [
            b.get("name")
            for b in wrapper.iter("body")
            if b.get("name", "").startswith("gripper_" + side + "_")
        ]
        for i, a in enumerate(hand):
            for b in hand[i + 1 :]:
                sub(contacts, "exclude", body1=a, body2=b)
        base = wrapper.find(f".//body[@name='gripper_{side}_base_link']")
        sub(
            base,
            "site",
            name=side + "_tcp",
            pos="0 0 0.19",
            size="0.008",
            rgba="0 1 0 1",
        )
    sub(world, "light", pos="0 -2 5", dir="0 0 -1", diffuse=".8 .8 .8")
    sub(world, "camera", name="overview", pos="4 -6 4", xyaxes=".85 .53 0 -.23 .36 .9")
    # Use the same generated workstation, excluding decorative non-collision parts.
    preview = json.loads((OUT / "preview.json").read_text())
    meta = preview["meta"]
    for o in preview["objects"]:
        name = o["name"]
        if (
            "/Robot/" in name
            or "/Parcels/" in name
            or any(t in name for t in ["Grid", "Safety", "Tread", "Tape"])
        ):
            continue
        if o["kind"] not in ["box", "cylinder"]:
            continue
        T = np.array(o["matrix"]).reshape(4, 4, order="F")
        size = (
            np.array(o["size"]) / 2
            if o["kind"] == "box"
            else [o["size"][0], o["size"][1] / 2]
        )
        sub(
            world,
            "geom",
            name=name.removeprefix("/World/").replace("/", "_"),
            type=o["kind"],
            size=fmt(size),
            pos=fmt(T[:3, 3]),
            quat=quat(T[:3, :3]),
            rgba=fmt(o["color"] + [1]),
            group="0",
        )
    for p in meta["parcels"]:
        b = sub(
            world,
            "body",
            name=p["id"],
            pos=fmt(p["position_m"]),
            quat=fmt([math.cos(p["yaw_rad"] / 2), 0, 0, math.sin(p["yaw_rad"] / 2)]),
        )
        sub(b, "freejoint", name=p["id"] + "_free")
        sub(
            b,
            "geom",
            name=p["id"] + "_collision",
            type="box",
            size=fmt(np.array(p["dimensions_m"]) / 2),
            mass=p["mass_kg"],
            rgba=".65 .43 .23 1",
            group="2",
        )
        asset = mj.find("asset")
        tex = p["id"] + "_label_texture"
        mat = p["id"] + "_label_material"
        sub(
            asset,
            "texture",
            name=tex,
            type="2d",
            file="labels/" + p["tracking_id"] + ".png",
        )
        sub(
            asset,
            "material",
            name=mat,
            texture=tex,
            texrepeat="1 1",
            texuniform="false",
            rgba="1 1 1 1",
        )
        L = np.array(p["label_transform_local"])
        a, bb = p["label_size_m"]
        sub(
            b,
            "geom",
            name=p["id"] + "_label",
            type="box",
            size=fmt([a / 2, bb / 2, 0.0001]),
            pos=fmt(L[:3, 3]),
            quat=quat(L[:3, :3]),
            material=mat,
            contype="0",
            conaffinity="0",
            mass="0",
            group="3",
        )
    # Calibration parcel: known dimensions and mass, grasped by contact only.
    b = sub(world, "body", name="test_parcel", pos="-.55 -.42 .877")
    sub(b, "freejoint", name="test_parcel_free")
    sub(
        b,
        "geom",
        name="test_parcel_geom",
        type="box",
        size=".025 .025 .025",
        mass=".1",
        rgba=".9 .3 .12 1",
        friction="1 .02 .002",
        group="2",
    )
    sub(
        b,
        "site",
        name="test_label",
        pos="0 0 .026",
        size=".015 .015 .001",
        type="box",
        rgba="1 1 1 1",
    )
    manifest = configure_workcell(mj, meta["parcels"])
    (OUT / "workstation_manifest.json").write_text(json.dumps(manifest, indent=2))
    configuration["workstation_model"] = "scenes/g2_operation.xml"
    configuration["base_position_m"] = manifest["base_position_m"]
    E.indent(mj)
    E.ElementTree(mj).write(OUT / "g2_operation.xml", encoding="unicode")
    m = mujoco.MjModel.from_xml_path(str(OUT / "g2_operation.xml"))
    (OUT / "g2_configuration.json").write_text(json.dumps(configuration, indent=2))
    print(
        "Compiled",
        m.nbody,
        "bodies",
        m.njnt,
        "joints",
        m.nu,
        "actuators",
        m.neq,
        "mimic constraints",
    )


if __name__ == "__main__":
    main()
