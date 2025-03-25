import mujoco
from pprint import pprint
import numpy as np

def quaternion_to_rotation_matrix(q):
    w, x, y, z = q
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z

    return np.array([
        [1 - 2*(yy + zz),     2*(xy - wz),       2*(xz + wy)],
        [2*(xy + wz),         1 - 2*(xx + zz),   2*(yz - wx)],
        [2*(xz - wy),         2*(yz + wx),       1 - 2*(xx + yy)]
    ])

assembled_model = mujoco.MjSpec.from_file("bloks_test/test_blocks/test_blocks.xml")
diff_body = mujoco.MjSpec.from_file("bloks_test/test_blocks_assy/test_blocks_assy.xml")

body = diff_body.bodies[2]

R = quaternion_to_rotation_matrix(body.iquat)
I = np.diag(body.inertia)
I_tot = R @ I @ R.T
print(body.name)
print(I_tot)

mass_total = 0
ipos = np.array([0., 0., 0.])
pos = np.array([0., 0., 0.])
I_tot = np.zeros((3,3))

for body in diff_body.bodies[1:]:
    mass_total += body.mass
    pos += quaternion_to_rotation_matrix(body.quat) @ body.pos
    ipos +=  body.mass * (pos + body.ipos)
    I = np.diag(body.inertia)
    I_body = R @ I @ R.T
    print(body.name, body.mass, body.ipos)
    print(I_body)
    print()

ipos = ipos / mass_total
pos = np.array([0., 0., 0.])
for body in diff_body.bodies[1:]:
    mass_total += body.mass
    pos += quaternion_to_rotation_matrix(body.quat) @ body.pos
    ipos_curr = pos + body.ipos
    I = np.diag(body.inertia)
    I_body = R @ I @ R.T
    d = (ipos_curr - ipos)
    d_squared = np.dot(d, d)
    d_outer = np.outer(d, d)
    I_tot += I_body + body.mass * (d_squared * np.eye(3) - d_outer)

print("Full robot", mass_total, ipos)
print(I_tot)

# pprint(dir(diff_body))