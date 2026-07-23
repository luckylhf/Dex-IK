"""Dual-arm inverse kinematics for Dex."""

from __future__ import annotations

import os
import platform
import subprocess
import tempfile
import time
from pathlib import Path
from typing import Dict, Optional, Tuple

import casadi
import numpy as np


LEFT_ARM_JOINTS = (
    "shoulder_pitch_l_joint",
    "shoulder_roll_l_joint",
    "shoulder_yaw_l_joint",
    "elbow_pitch_l_joint",
    "elbow_yaw_l_joint",
    "wrist_pitch_l_joint",
    "wrist_roll_l_joint",
)

RIGHT_ARM_JOINTS = (
    "shoulder_pitch_r_joint",
    "shoulder_roll_r_joint",
    "shoulder_yaw_r_joint",
    "elbow_pitch_r_joint",
    "elbow_yaw_r_joint",
    "wrist_pitch_r_joint",
    "wrist_roll_r_joint",
)


def _load_pinocchio():
    try:
        import pinocchio as pin
        from pinocchio import casadi as cpin
    except (ImportError, ModuleNotFoundError) as exc:
        raise ImportError(
            "Dex IK requires Pinocchio with CasADi support. "
            "Install the conda-forge environment from environment.yml."
        ) from exc

    if not hasattr(pin, "buildModelFromUrdf"):
        raise ImportError(
            "The imported 'pinocchio' module is not the robotics library. "
            "Use the conda-forge pinocchio package from environment.yml."
        )
    return pin, cpin


def default_urdf_path() -> Path:
    return (
        Path(__file__).resolve().parent
        / "assets"
        / "dex_description"
        / "urdf"
        / "dex.urdf"
    )


def shared_library_suffix() -> str:
    system = platform.system()
    if system == "Linux":
        return ".so"
    if system == "Darwin":
        return ".dylib"
    raise RuntimeError(f"Dynamic solver generation is unsupported on {system}.")


class DexArmIK:
    """Solve simultaneous 6D poses for the two Dex TCP frames."""

    def __init__(
        self,
        urdf_path: Optional[os.PathLike] = None,
        *,
        jit: bool = False,
        shared_library: Optional[os.PathLike] = None,
    ) -> None:
        self._pin, self._cpin = _load_pinocchio()
        self.urdf_path = Path(urdf_path) if urdf_path else default_urdf_path()
        if not self.urdf_path.is_file():
            raise FileNotFoundError(f"Dex URDF not found: {self.urdf_path}")

        full_model = self._pin.buildModelFromUrdf(str(self.urdf_path))
        arm_joint_set = set(LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS)
        missing = arm_joint_set.difference(full_model.names)
        if missing:
            raise ValueError(f"Dex URDF is missing arm joints: {sorted(missing)}")

        joints_to_lock = [
            joint_id
            for joint_id, name in enumerate(full_model.names)
            if joint_id > 0 and name not in arm_joint_set
        ]
        self.model = self._pin.buildReducedModel(
            full_model,
            joints_to_lock,
            self._pin.neutral(full_model),
        )
        self.data = self.model.createData()

        if self.model.nq != 14 or self.model.nv != 14:
            raise ValueError(
                f"Expected a 14-DoF dual-arm model, got nq={self.model.nq}, "
                f"nv={self.model.nv}."
            )

        self.left_tcp_frame = self._frame_id("left_tcp_link")
        self.right_tcp_frame = self._frame_id("right_tcp_link")
        self.joint_indices = {
            name: self.model.joints[self.model.getJointId(name)].idx_q
            for name in self.model.names[1:]
        }
        self.joint_names = tuple(
            name
            for name, _ in sorted(
                self.joint_indices.items(), key=lambda item: item[1]
            )
        )
        self._q = self._pin.neutral(self.model)
        self._jit = jit

        if shared_library is None:
            self._setup_solver()
        else:
            library_path = Path(shared_library)
            if not library_path.is_file():
                raise FileNotFoundError(
                    f"Generated IK library not found: {library_path}"
                )
            self._solver = casadi.external("dex_ik_solver", str(library_path))

    def _frame_id(self, name: str) -> int:
        if not self.model.existFrame(name):
            raise ValueError(f"Dex URDF is missing frame: {name}")
        return self.model.getFrameId(name)

    def _setup_solver(self) -> None:
        cmodel = self._cpin.Model(self.model)
        cdata = cmodel.createData()

        q = casadi.SX.sym("q", self.model.nq)
        q_init = casadi.SX.sym("q_init", self.model.nq)
        left_target = casadi.SX.sym("left_target", 4, 4)
        right_target = casadi.SX.sym("right_target", 4, 4)

        self._cpin.framesForwardKinematics(cmodel, cdata, q)
        left_pose = cdata.oMf[self.left_tcp_frame]
        right_pose = cdata.oMf[self.right_tcp_frame]

        left_position_error = left_pose.translation - left_target[:3, 3]
        right_position_error = right_pose.translation - right_target[:3, 3]
        left_rotation_error = self._cpin.log3(
            left_pose.rotation @ left_target[:3, :3].T
        )
        right_rotation_error = self._cpin.log3(
            right_pose.rotation @ right_target[:3, :3].T
        )

        cost = (
            60.0
            * (
                casadi.sumsqr(left_position_error)
                + casadi.sumsqr(right_position_error)
            )
            + 3.0
            * (
                casadi.sumsqr(left_rotation_error)
                + casadi.sumsqr(right_rotation_error)
            )
            + 1e-4 * casadi.sumsqr(q)
            + 0.01 * casadi.sumsqr(q - q_init)
        )

        parameters = casadi.vertcat(
            casadi.vec(left_target),
            casadi.vec(right_target),
            q_init,
        )
        options = {
            "ipopt": {
                "print_level": 0,
                "max_iter": 30,
                "tol": 1e-4,
                "acceptable_tol": 1e-3,
                "linear_solver": "mumps",
            },
            "print_time": False,
            "expand": True,
            "jit": self._jit,
            "compiler": "shell",
            "jit_options": {"flags": ["-O3"], "verbose": False},
        }
        nlp_solver = casadi.nlpsol(
            "dex_nlp_solver",
            "ipopt",
            {"x": q, "f": cost, "p": parameters},
            options,
        )
        result = nlp_solver(
            x0=q_init,
            p=parameters,
            lbx=self.model.lowerPositionLimit,
            ubx=self.model.upperPositionLimit,
        )
        self._solver = casadi.Function(
            "dex_ik_solver",
            [left_target, right_target, q_init],
            [result["x"]],
            ["left_target", "right_target", "q_init"],
            ["q"],
        )

    @staticmethod
    def _target_matrix(target: np.ndarray, name: str) -> np.ndarray:
        matrix = np.asarray(target, dtype=float)
        if matrix.shape != (4, 4):
            raise ValueError(f"{name} must be a 4x4 homogeneous matrix.")
        return matrix

    def _configuration(self, q: np.ndarray) -> np.ndarray:
        configuration = np.asarray(q, dtype=float).reshape(-1)
        if configuration.shape != (self.model.nq,):
            raise ValueError(
                f"q must contain {self.model.nq} joint positions, "
                f"got {configuration.size}."
            )
        return configuration

    def get_arm_joint_positions(self, q: np.ndarray, side: str) -> np.ndarray:
        """Return the seven joint positions for side ``left`` or ``right``."""
        configuration = self._configuration(q)
        if side == "left":
            names = LEFT_ARM_JOINTS
        elif side == "right":
            names = RIGHT_ARM_JOINTS
        else:
            raise ValueError("side must be 'left' or 'right'.")
        return np.array(
            [configuration[self.joint_indices[name]] for name in names]
        )

    def forward_kinematics(self, q: np.ndarray) -> Dict[str, np.ndarray]:
        """Return homogeneous poses for the left and right TCP frames."""
        configuration = self._configuration(q)
        self._pin.framesForwardKinematics(self.model, self.data, configuration)
        return {
            "left": self.data.oMf[self.left_tcp_frame].homogeneous.copy(),
            "right": self.data.oMf[self.right_tcp_frame].homogeneous.copy(),
        }

    def pose_errors(
        self,
        q: np.ndarray,
        left_target: np.ndarray,
        right_target: np.ndarray,
    ) -> Tuple[float, float, float, float]:
        """Return left/right position and rotation errors in metres/radians."""
        left_target = self._target_matrix(left_target, "left_target")
        right_target = self._target_matrix(right_target, "right_target")
        current = self.forward_kinematics(q)
        left_position = np.linalg.norm(
            current["left"][:3, 3] - left_target[:3, 3]
        )
        right_position = np.linalg.norm(
            current["right"][:3, 3] - right_target[:3, 3]
        )
        left_rotation = np.linalg.norm(
            self._pin.log3(current["left"][:3, :3].T @ left_target[:3, :3])
        )
        right_rotation = np.linalg.norm(
            self._pin.log3(current["right"][:3, :3].T @ right_target[:3, :3])
        )
        return left_position, right_position, left_rotation, right_rotation

    def solve_ik(
        self,
        left_target: np.ndarray,
        right_target: np.ndarray,
        *,
        q_init: Optional[np.ndarray] = None,
        max_iterations: int = 5,
        position_tolerance: float = 1e-4,
        orientation_tolerance: float = 1e-4,
        verbose: bool = False,
    ) -> np.ndarray:
        """Solve the simultaneous left and right TCP poses."""
        left_target = self._target_matrix(left_target, "left_target")
        right_target = self._target_matrix(right_target, "right_target")
        if q_init is not None:
            self._q = self._configuration(q_init).copy()

        for iteration in range(1, max_iterations + 1):
            started = time.perf_counter()
            try:
                result = self._solver(
                    casadi.DM(left_target),
                    casadi.DM(right_target),
                    casadi.DM(self._q),
                )
            except RuntimeError as exc:
                raise RuntimeError(
                    f"Dex IK failed on outer iteration {iteration}."
                ) from exc

            self._q = np.asarray(result, dtype=float).reshape(-1)
            errors = self.pose_errors(self._q, left_target, right_target)

            if verbose:
                elapsed_ms = (time.perf_counter() - started) * 1000.0
                print(
                    f"IK solve: {elapsed_ms:.2f} ms | "
                    f"iteration: {iteration}/{max_iterations} | "
                    f"position L/R: {errors[0] * 1000:.3f}/"
                    f"{errors[1] * 1000:.3f} mm | "
                    f"rotation L/R: {errors[2]:.5f}/{errors[3]:.5f} rad"
                )

            if (
                max(errors[:2]) <= position_tolerance
                and max(errors[2:]) <= orientation_tolerance
            ):
                break

        return self._q.copy()

    def export_shared_library(
        self,
        output: Optional[os.PathLike] = None,
        *,
        conda_prefix: Optional[os.PathLike] = None,
        compiler: Optional[str] = None,
    ) -> Path:
        """Generate the platform-native CasADi solver library."""
        suffix = shared_library_suffix()
        output_path = (
            Path(output)
            if output is not None
            else Path(".casadi_cache") / f"dex_ik_solver{suffix}"
        )
        if output_path.suffix != suffix:
            raise ValueError(
                f"{platform.system()} solver libraries must use {suffix}."
            )
        output_path = output_path.resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)

        prefix = Path(
            conda_prefix
            or os.environ.get("CONDA_PREFIX")
            or Path(os.sys.prefix)
        ).resolve()
        casadi_dir = Path(casadi.__file__).resolve().parent
        cc = compiler or os.environ.get("CC") or "cc"

        with tempfile.TemporaryDirectory(prefix="dex_ik_build_") as temp_dir:
            temp_path = Path(temp_dir)
            previous_directory = Path.cwd()
            try:
                os.chdir(temp_path)
                generator = casadi.CodeGenerator("dex_ik_solver.c")
                generator.add(self._solver.expand())
                generator.generate()
            finally:
                os.chdir(previous_directory)

            shared_flag = "-dynamiclib" if platform.system() == "Darwin" else "-shared"
            cpp_library = "-lc++" if platform.system() == "Darwin" else "-lstdc++"
            command = [
                cc,
                "-fPIC",
                shared_flag,
                "-O3",
                str(temp_path / "dex_ik_solver.c"),
                "-Dcasadi_inf=INFINITY",
                "-o",
                str(output_path),
                f"-I{prefix / 'include'}",
                f"-I{casadi_dir / 'include'}",
                f"-L{casadi_dir}",
                f"-L{prefix / 'lib'}",
                f"-Wl,-rpath,{casadi_dir}",
                f"-Wl,-rpath,{prefix / 'lib'}",
                "-lcasadi",
                "-lipopt",
                cpp_library,
                "-lm",
            ]
            subprocess.run(command, check=True)

        return output_path
