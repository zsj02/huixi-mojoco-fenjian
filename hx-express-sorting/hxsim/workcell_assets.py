"""Configure the one shared workcell, including its operation bay and labelled target."""

import sys, pathlib, json, hashlib, xml.etree.ElementTree as E

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from hxsim.runtime import ROOT
from PIL import Image, ImageDraw
from barcode import Code128
from barcode.writer import ImageWriter
from hxsim.conveyor_assets import install_conveyor


def configure_workcell(root, parcels):
    root.set("model", "HX G2 shared sorting workstation")
    world, assets = root.find("worldbody"), root.find("asset")
    # Keep all six incoming parcels on the table, leaving a reachable front-right operation bay.
    layout = [
        (-1.0, -0.25),
        (-0.66, -0.25),
        (-1.0, 0.10),
        (-0.66, 0.02),
        (-0.66, 0.29),
        (-0.28, 0.26),
    ]
    parcels = json.loads(json.dumps(parcels))
    for parcel, xy in zip(parcels, layout):
        parcel["position_m"][:2] = xy
        root.find(f'.//body[@name="{parcel["id"]}"]').set(
            "pos", " ".join(map(str, parcel["position_m"]))
        )
    root.find('.//body[@name="g2_mount"]').set("pos", "-0.25 -0.85 0.046")
    # Keep the loading bay clear; conveyor_assets installs the unified near-side control box.
    rail = root.find('.//geom[@name="Conveyor_GuideL"]')
    rail.set("size", "1.22 .0125 .0175")
    rail.set("pos", "1.97 -.38 .89")
    body = root.find('.//body[@name="test_parcel"]')
    body.set("pos", "-.1 -.32 .902")
    body.set("quat", "0 1 0 0")
    body.find("geom").set("size", ".025 .035 .05")
    body.find("geom").set("rgba", ".65 .43 .23 1")
    body.find("site").set("pos", "0 0 .0502")
    body.find("site").set("rgba", "0 0 0 0")
    E.SubElement(
        assets,
        "texture",
        name="target_label_texture",
        type="2d",
        file="labels/HX1001.png",
    )
    E.SubElement(
        assets,
        "material",
        name="target_label_material",
        texture="target_label_texture",
        texrepeat="1 1",
        texuniform="false",
        rgba="1 1 1 1",
    )
    E.SubElement(
        body,
        "geom",
        name="test_parcel_label",
        type="box",
        size=".020 .030 .0001",
        pos="0 0 .0501",
        material="target_label_material",
        contype="0",
        conaffinity="0",
        mass="0",
        group="3",
    )
    # Cameras use OpenGL axes; the adapter converts to optical +X right, +Y down, +Z forward.
    E.SubElement(
        world,
        "camera",
        name="workspace",
        pos=".15 -.20 2.6",
        xyaxes="1 0 0 0 1 0",
        fovy="50",
    )
    E.SubElement(
        world,
        "camera",
        name="inspection",
        pos=".28 -.22 1.5",
        xyaxes="1 0 0 0 1 0",
        fovy="16",
    )
    label = Image.new("RGB", (600, 900), "white")
    draw = ImageDraw.Draw(label)
    draw.text((30, 55), "HX LAB / SIMULATION ONLY", fill="black", font_size=30)
    draw.text((30, 145), "HX  -  HX1001", fill="black", font_size=38)
    draw.text((30, 240), "50 x 70 x 100 mm / 100 g", fill="black", font_size=25)
    bars = Code128("HX1001", writer=ImageWriter()).render(
        {"module_width": 0.3, "module_height": 16, "quiet_zone": 3, "write_text": False}
    )
    label.paste(bars.resize((560, 290), Image.Resampling.NEAREST), (20, 430))
    draw.text((35, 790), "TEST LABEL / NOT FOR SHIPPING", fill="black", font_size=25)
    label.save(ROOT / "scenes/labels/HX1001.png")
    manifest = {
        "id": "test_parcel",
        "packaging": "carton",
        "dimensions_m": [0.05, 0.07, 0.1],
        "mass_kg": 0.1,
        "contents": "Small boxed sample; rigid-body approximation",
        "tracking_id": "HX1001",
        "initial_label_face": "down",
        "label_normal_local": [0, 0, 1],
        "position_m": [-0.1, -0.32, 0.902],
        "yaw_rad": 0,
        "label_transform_local": [
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0.0501],
            [0, 0, 0, 1],
        ],
        "initial_quaternion_wxyz": [0, 1, 0, 0],
    }
    install_conveyor(root)
    return {
        "scene_model": "scenes/g2_operation.xml",
        "parcels": [*parcels, manifest],
        "task_target": "test_parcel",
        "base_position_m": [-0.25, -0.85, 0.046],
        "description": "One shared workcell for index, MuJoCo and Harness; all seven parcels remain present.",
    }
