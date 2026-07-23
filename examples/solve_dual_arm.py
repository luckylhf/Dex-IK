"""Solve a reachable dual-arm pose and verify it with forward kinematics."""

import numpy as np

from dex_ik import DexArmIK


solver = DexArmIK(jit=True)

known_configuration = np.array(
    [
        0.20,
        0.30,
        -0.20,
        -0.60,
        0.20,
        0.10,
        -0.10,
        -0.20,
        -0.30,
        0.20,
        -0.60,
        -0.20,
        0.10,
        0.10,
    ]
)
targets = solver.forward_kinematics(known_configuration)
solution = solver.solve_ik(
    targets["left"],
    targets["right"],
    verbose=True,
)
errors = solver.pose_errors(solution, targets["left"], targets["right"])

print("Joint order:", solver.joint_names)
print("Solution:", solution)
print("Position errors L/R (m):", errors[:2])
print("Rotation errors L/R (rad):", errors[2:])
