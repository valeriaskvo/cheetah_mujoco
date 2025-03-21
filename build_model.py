import mujoco
from pprint import pprint

def find_mount(spec, mount_name):
    def _find_mount_recursive(body):
        for site in body.sites:
            if mount_name in site.name:
                return body, site

        for child in body.bodies:
            result = _find_mount_recursive(child)
            if result:
                return result
        return None

    result = _find_mount_recursive(spec.worldbody)
    if not result:
        raise ValueError(f"Mount point '{mount_name}' not found")
    return result

scene = mujoco.MjSpec.from_file("cheetah_assets/scene.xml")
torso = mujoco.MjSpec.from_file("cheetah_assets/torso.xml")
fl_leg = mujoco.MjSpec.from_file("cheetah_assets/fl_leg.xml")
fr_leg = mujoco.MjSpec.from_file("cheetah_assets/fr_leg.xml")

body = fl_leg.worldbody.bodies[0]
pprint(dir(body))
print(body.childclass)

# parent_body, mount_site = find_mount(torso, 'fl_hip_mount')
# mount_site.attach_body(fl_leg.bodies[1])

# for joint in torso.joints:
#     if joint.type == mujoco.mjtJoint.mjJNT_FREE:
#         joint.delete()
# torso.compile()

# xml_content = torso.to_xml()
# with open("cheetah_assets/cheetah_assy.xml", "w") as f:
#     f.write(xml_content)