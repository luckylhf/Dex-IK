import os
import tempfile
import unittest
from pathlib import Path

import numpy as np

from dex_ik import DexArmIK
from dex_ik.solver import LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS, shared_library_suffix


def robotics_pinocchio_available():
    try:
        import pinocchio as pin
        from pinocchio import casadi as unused_cpin
    except (ImportError, ModuleNotFoundError):
        return False
    return hasattr(pin, "buildModelFromUrdf")


@unittest.skipUnless(
    robotics_pinocchio_available(),
    "conda-forge Pinocchio with CasADi support is not installed",
)
class DexSolverTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.solver = DexArmIK()
        cls.goals = {
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

    def test_reduced_model_contains_only_dual_arm_joints(self):
        self.assertEqual(self.solver.model.nq, 14)
        self.assertEqual(
            self.solver.joint_names,
            LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS,
        )

    def test_reachable_pose_round_trip(self):
        for name, goal in self.goals.items():
            with self.subTest(pose=name):
                targets = self.solver.forward_kinematics(goal)
                solution = self.solver.solve_ik(
                    targets["left"],
                    targets["right"],
                    q_init=np.zeros(14),
                    max_iterations=5,
                )
                errors = self.solver.pose_errors(
                    solution,
                    targets["left"],
                    targets["right"],
                )
                self.assertLess(max(errors[:2]), 5e-4)
                self.assertLess(max(errors[2:]), 5e-4)

    @unittest.skipUnless(
        os.environ.get("DEX_IK_TEST_SHARED_LIBRARY") == "1",
        "set DEX_IK_TEST_SHARED_LIBRARY=1 to test native solver generation",
    )
    def test_generated_library_can_solve(self):
        for name, goal in self.goals.items():
            with self.subTest(pose=name):
                targets = self.solver.forward_kinematics(goal)
                with tempfile.TemporaryDirectory(prefix="dex_ik_test_") as temp_dir:
                    library = Path(temp_dir) / f"dex_ik_solver{shared_library_suffix()}"
                    self.solver.export_shared_library(library)
                    native_solver = DexArmIK(shared_library=library)
                    solution = native_solver.solve_ik(
                        targets["left"],
                        targets["right"],
                        q_init=np.zeros(14),
                        max_iterations=5,
                    )
                    errors = native_solver.pose_errors(
                        solution,
                        targets["left"],
                        targets["right"],
                    )
                    self.assertLess(max(errors[:2]), 5e-4)
                    self.assertLess(max(errors[2:]), 5e-4)


if __name__ == "__main__":
    unittest.main()
