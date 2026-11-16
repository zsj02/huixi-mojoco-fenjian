"""Finite-force moving belt segments and a physical start button in the shared scene."""

import xml.etree.ElementTree as E


def install_conveyor(root):
    world = root.find("worldbody")
    act = root.find("actuator")
    # Flush transfer avoids tipping tall cartons over the old 6/15 mm steps.
    root.find('.//geom[@name="Cart_ReceivingTray"]').set("pos", "3.75 0 .8325")
    root.find('.//geom[@name="Cart_Bridge"]').set("pos", "3.18 0 .844")
    # A visual backing remains; support/friction comes from the driven slats.
    backing = root.find('.//geom[@name="Conveyor_Belt"]')
    backing.set("contype", "0")
    backing.set("conaffinity", "0")
    backing.set("size", "1.5 .34 .020")
    backing.set("pos", "1.65 0 .820")
    for g in root.findall(".//geom"):
        if g.get("group") == "1":
            g.set("conaffinity", "25")  # world, belt, start button
        if g.get("group") == "2":
            g.set("conaffinity", "9")  # world and belt
    for i in range(30):
        body = E.SubElement(
            world, "body", name=f"belt_slat_{i:02d}", pos=f"{.2+.1*i:.5f} 0 .8425"
        )
        E.SubElement(
            body,
            "joint",
            name=f"belt_slide_{i:02d}",
            type="slide",
            axis="1 0 0",
            limited="false",
            damping="0",
            armature=".01",
            frictionloss="0",
        )
        E.SubElement(
            body,
            "geom",
            name=f"Conveyor_Slat_{i:02d}",
            type="box",
            size=".0505 .34 .0075",
            mass="1",
            friction=".8 .01 .001",
            contype="8",
            conaffinity="0",
            group="0",
            rgba=".09 .13 .16 1" if i % 2 else ".12 .17 .20 1",
        )
        E.SubElement(
            act,
            "velocity",
            name=f"belt_drive_{i:02d}",
            joint=f"belt_slide_{i:02d}",
            kv="300",
            ctrlrange="0 .12",
            ctrllimited="true",
            forcerange="-80 80",
            forcelimited="true",
        )
    # Replace both legacy loose controls. One parent transform carries enclosure,
    # mount, collars and both physical buttons, so relocation cannot orphan a cap.
    for name in ("Conveyor_Control", "Conveyor_EStop"):
        old = world.find(f'geom[@name="{name}"]')
        if old is not None:
            world.remove(old)
    # Side-mounted enclosure: the entire assembly is below the .85 m belt
    # surface and outside its near edge. Both caps face the robot (-Y).
    station = E.SubElement(
        world, "body", name="Conveyor_ControlStation", pos=".24 -.46 .78"
    )
    E.SubElement(
        station,
        "geom",
        name="Conveyor_Control",
        type="box",
        size=".085 .035 .045",
        rgba=".15 .22 .24 1",
        group="0",
    )
    E.SubElement(
        station,
        "geom",
        name="Conveyor_ControlMount",
        type="box",
        pos="0 .06 0",
        size=".055 .025 .018",
        rgba=".4 .45 .48 1",
        group="0",
    )
    E.SubElement(
        station,
        "geom",
        name="Conveyor_ControlBracket",
        type="box",
        pos="0 .085 0",
        size=".075 .008 .035",
        rgba=".4 .45 .48 1",
        group="0",
    )
    for key, x, cap, radius, color in [
        ("Start", -0.04, "Conveyor_StartCap", 0.023, ".08 .8 .2 1"),
        ("Stop", 0.04, "Conveyor_EStop", 0.028, "1 .12 .07 1"),
    ]:
        E.SubElement(
            station,
            "geom",
            name=f"Conveyor_{key}Collar",
            type="cylinder",
            pos=f"{x} -.038 0",
            size=f"{radius+.007} .003",
            quat=".70710678 .70710678 0 0",
            rgba=".95 .72 .08 1" if key == "Stop" else ".48 .55 .57 1",
            contype="0",
            conaffinity="0",
            mass="0",
            group="0",
        )
        button = E.SubElement(
            station,
            "body",
            name=f"Conveyor_{key}Button",
            pos=f"{x} -.054 0",
            gravcomp="1",
        )
        E.SubElement(
            button,
            "joint",
            name=f"conveyor_{key.lower()}_switch",
            type="slide",
            axis="0 1 0",
            limited="true",
            range="0 .008",
            stiffness="400",
            damping="3",
            armature=".001",
            margin="0",
        )
        E.SubElement(
            button,
            "geom",
            name=f"Conveyor_{key}Stem",
            type="cylinder",
            pos="0 .011 0",
            size=".009 .009",
            quat=".70710678 .70710678 0 0",
            mass=".01",
            contype="0",
            conaffinity="0",
            rgba=".3 .35 .36 1",
            group="0",
        )
        E.SubElement(
            button,
            "geom",
            name=cap,
            type="cylinder",
            size=f"{radius} .008",
            quat=".70710678 .70710678 0 0",
            mass=".03",
            contype="16",
            conaffinity="0",
            rgba=color,
            group="0",
        )
    # Fixed scan cameras, not ground-truth-following cameras.
    for name, pos, fovy in [
        ("scan_hand", "-.1 -.32 1.75", "16"),
        ("scan_table_left", "-1 -.25 1.6", "22"),
        ("scan_table_right", "-.55 .04 1.8", "45"),
    ]:
        E.SubElement(
            world, "camera", name=name, pos=pos, xyaxes="1 0 0 0 1 0", fovy=fovy
        )
    E.SubElement(
        world,
        "camera",
        name="scan_front",
        pos="-.66 -1.1 1.1",
        xyaxes="1 0 0 0 .18 .984",
        fovy="18",
    )
