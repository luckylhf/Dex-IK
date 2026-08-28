"""Validate the Dex Python and native IK solvers."""

import argparse
import platform
import time
from pathlib import Path

import numpy as np

from dex_ik import DexArmIK


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Build and validate the Dex native IK library."
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Output .so on Linux or .dylib on macOS.",
    )
    args = parser.parse_args()

    position_tolerance = 5e-4
    orientation_tolerance = 5e-4
    goals = {
        "hands_up": np.array(
            [-0.8, 0.3, 0.0, 0.0, -0.1, 0.1, 0.1,
             -0.8, -0.3, 0.0, 0.0, 0.1, 0.1, 0.1]
        ),
        "arms_forward": np.array(
            [0.3, 0.4, -0.3, -0.8, 0.2, 0.1, -0.1,
             -0.3, -0.4, 0.3, -0.8, -0.2, 0.1, 0.1]
        ),
        "arms_down": np.array(
            [0.5, -0.1, 0.0, -2.0, 0.0, 0.0, 0.0,
             -0.5, 0.1, 0.0, -2.0, 0.0, 0.0, 0.0]
        ),
    }

    print(f"Platform: {platform.system()} {platform.machine()}")
    print("Position tolerance: 0.500 mm")
    print("Orientation tolerance: 0.000500 rad")

    solver = DexArmIK()
    targets = {
        name: solver.forward_kinematics(goal)
        for name, goal in goals.items()
    }
    for name, target in targets.items():
        started = time.perf_counter()
        solution = solver.solve_ik(
            target["left"],
            target["right"],
            q_init=np.zeros(14),
        )
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        errors = solver.pose_errors(solution, target["left"], target["right"])
        if (
            max(errors[:2]) > position_tolerance
            or max(errors[2:]) > orientation_tolerance
        ):
            raise RuntimeError(f"[{name}] Python solver accuracy failed: {errors}")
        print(
            f"[{name}] Python PASS: {elapsed_ms:.2f} ms, "
            f"position={max(errors[:2]) * 1000.0:.3f} mm, "
            f"orientation={max(errors[2:]):.6f} rad"
        )

    library = solver.export_shared_library(args.output)
    print(f"Generated native library: {library}")
    native_solver = DexArmIK(shared_library=library)
    for name, target in targets.items():
        started = time.perf_counter()
        solution = native_solver.solve_ik(
            target["left"],
            target["right"],
            q_init=np.zeros(14),
        )
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        errors = native_solver.pose_errors(
            solution,
            target["left"],
            target["right"],
        )
        if (
            max(errors[:2]) > position_tolerance
            or max(errors[2:]) > orientation_tolerance
        ):
            raise RuntimeError(f"[{name}] native solver accuracy failed: {errors}")
        print(
            f"[{name}] native PASS: {elapsed_ms:.2f} ms, "
            f"position={max(errors[:2]) * 1000.0:.3f} mm, "
            f"orientation={max(errors[2:]):.6f} rad"
        )

    print("PASS: all Python and native Dex IK checks passed.")


if __name__ == "__main__":
    main()
