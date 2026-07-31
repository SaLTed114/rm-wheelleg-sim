#!/usr/bin/env python3
"""Extract reduced LQR parameters from the closed-chain MuJoCo model."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import least_squares


@dataclass(frozen=True)
class SideSpec:
    name: str
    joint_names: tuple[str, ...]
    loop_sites: tuple[tuple[str, str], ...]
    hip_site: str
    wheel_site: str
    wheel_body: str
    wheel_joint: str
    wheel_collision: str
    leg_bodies: tuple[str, ...]


@dataclass
class PoseSolution:
    length: float
    joint_position: np.ndarray
    closure_error: float
    target_error: float
    joint_margin: float
    feasible: bool


SIDE_SPECS = (
    SideSpec(
        name="left",
        joint_names=(
            "Right_front_joint", "Right_front_child1_joint",
            "Right_front_child2_joint", "Right_front_joint3_joint",
            "Right_rear_joint", "Right_rear_child1_joint",
        ),
        loop_sites=(
            ("Right_front_site1", "Right_rear_site1"),
            ("Right_front_site2", "Right_rear_site2"),
        ),
        hip_site="Right_virtual_hip_site",
        wheel_site="Right_wheel_axis_site",
        wheel_body="Right_Wheel_link",
        wheel_joint="Right_Wheel_joint",
        wheel_collision="Right_wheel_collision",
        leg_bodies=(
            "Right_front_link", "Right_front_child1_link",
            "Right_front_child2_link", "Right_front_child3_link",
            "Right_rear_link", "Right_rear_child1_link",
        ),
    ),
    SideSpec(
        name="right",
        joint_names=(
            "Left_front_joint", "Left_front_child1_joint",
            "Left_front_child2_joint", "Left_front_child3_joint",
            "Left_rear_joint", "Left_rear_child1_joint",
        ),
        loop_sites=(
            ("Left_front_site1", "Left_rear_site1"),
            ("Left_front_site2", "Left_rear_site2"),
        ),
        hip_site="Left_virtual_hip_site",
        wheel_site="Left_wheel_axis_site",
        wheel_body="Left_Wheel_link",
        wheel_joint="Left_Wheel_joint",
        wheel_collision="Left_wheel_collision",
        leg_bodies=(
            "Left_front_link", "Left_front_child1_link",
            "Left_front_child2_link", "Left_front_child3_link",
            "Left_rear_link", "Left_rear_child1_link",
        ),
    ),
)


class ModelParameterExtractor:
    def __init__(self, model_path: Path):
        self.model_path = model_path
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.closure_limits: dict[str, float] = {}
        mujoco.mj_forward(self.model, self.data)

    def require_id(self, object_type: mujoco.mjtObj, name: str) -> int:
        object_id = mujoco.mj_name2id(self.model, object_type, name)
        if object_id < 0:
            raise ValueError(f"MuJoCo model is missing {name!r}")
        return object_id

    def _side_addresses(self, spec: SideSpec) -> dict[str, object]:
        joints = [
            self.require_id(mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in spec.joint_names
        ]
        return {
            "joints": joints,
            "qpos": [self.model.jnt_qposadr[joint] for joint in joints],
            "loops": [
                (
                    self.require_id(mujoco.mjtObj.mjOBJ_SITE, first),
                    self.require_id(mujoco.mjtObj.mjOBJ_SITE, second),
                )
                for first, second in spec.loop_sites
            ],
            "hip": self.require_id(mujoco.mjtObj.mjOBJ_SITE, spec.hip_site),
            "wheel": self.require_id(mujoco.mjtObj.mjOBJ_SITE, spec.wheel_site),
        }

    def _joint_bounds(self, joints: list[int]) -> tuple[np.ndarray, np.ndarray]:
        lower = []
        upper = []
        for joint in joints:
            if self.model.jnt_limited[joint]:
                lower.append(float(self.model.jnt_range[joint, 0]))
                upper.append(float(self.model.jnt_range[joint, 1]))
            else:
                lower.append(-np.pi)
                upper.append(np.pi)
        return np.asarray(lower), np.asarray(upper)

    def _pose_residual(
        self,
        position: np.ndarray,
        addresses: dict[str, object],
        target_length: float,
    ) -> np.ndarray:
        self.data.qpos[addresses["qpos"]] = position
        mujoco.mj_forward(self.model, self.data)
        residual = []
        for first, second in addresses["loops"]:
            residual.extend(self.data.site_xpos[first] - self.data.site_xpos[second])
        hip = self.data.site_xpos[addresses["hip"]]
        wheel = self.data.site_xpos[addresses["wheel"]]
        displacement = wheel - hip
        residual.extend((displacement[0], displacement[2] + target_length))
        return np.asarray(residual)

    def solve_pose(
        self,
        spec: SideSpec,
        target_length: float,
        initial_position: np.ndarray,
        closure_limit: float = 5.0e-4,
    ) -> PoseSolution:
        addresses = self._side_addresses(spec)
        lower, upper = self._joint_bounds(addresses["joints"])
        result = least_squares(
            lambda position: self._pose_residual(
                position, addresses, target_length),
            np.clip(initial_position, lower, upper),
            bounds=(lower, upper),
            xtol=1.0e-12,
            ftol=1.0e-12,
            gtol=1.0e-12,
            max_nfev=2000,
        )
        self._pose_residual(result.x, addresses, target_length)
        closure_error = max(
            float(np.linalg.norm(
                self.data.site_xpos[first] - self.data.site_xpos[second]))
            for first, second in addresses["loops"]
        )
        hip = self.data.site_xpos[addresses["hip"]]
        wheel = self.data.site_xpos[addresses["wheel"]]
        target = np.asarray((0.0, -target_length))
        target_error = float(np.linalg.norm((wheel - hip)[[0, 2]] - target))

        margins = []
        for index, joint in enumerate(addresses["joints"]):
            if self.model.jnt_limited[joint]:
                margins.append(min(
                    result.x[index] - self.model.jnt_range[joint, 0],
                    self.model.jnt_range[joint, 1] - result.x[index],
                ))
        joint_margin = float(min(margins)) if margins else float("inf")
        feasible = (
            result.success
            and closure_error <= closure_limit
            and target_error <= 5.0e-4
            and joint_margin >= 0.02
        )
        return PoseSolution(
            length=target_length,
            joint_position=result.x.copy(),
            closure_error=closure_error,
            target_error=target_error,
            joint_margin=joint_margin,
            feasible=feasible,
        )

    def scan_side(
        self,
        spec: SideSpec,
        lengths: np.ndarray,
        nominal_length: float,
    ) -> list[PoseSolution]:
        nominal_index = int(np.argmin(np.abs(lengths - nominal_length)))
        solutions: list[PoseSolution | None] = [None] * len(lengths)
        zero = np.zeros(len(spec.joint_names))
        nominal = self.solve_pose(spec, float(lengths[nominal_index]), zero)
        closure_limit = max(5.0e-4, nominal.closure_error + 5.0e-4)
        self.closure_limits[spec.name] = closure_limit
        nominal.feasible = (
            nominal.closure_error <= closure_limit
            and nominal.target_error <= 5.0e-4
            and nominal.joint_margin >= 0.02
        )
        solutions[nominal_index] = nominal

        position = nominal.joint_position
        for index in range(nominal_index - 1, -1, -1):
            solution = self.solve_pose(
                spec, float(lengths[index]), position, closure_limit)
            solutions[index] = solution
            position = solution.joint_position

        position = nominal.joint_position
        for index in range(nominal_index + 1, len(lengths)):
            solution = self.solve_pose(
                spec, float(lengths[index]), position, closure_limit)
            solutions[index] = solution
            position = solution.joint_position

        return [solution for solution in solutions if solution is not None]

    @staticmethod
    def safe_range(
        lengths: np.ndarray,
        side_solutions: tuple[list[PoseSolution], list[PoseSolution]],
        nominal_length: float,
    ) -> tuple[int, int]:
        feasible = np.asarray([
            side_solutions[0][index].feasible
            and side_solutions[1][index].feasible
            for index in range(len(lengths))
        ])
        nominal_index = int(np.argmin(np.abs(lengths - nominal_length)))
        if not feasible[nominal_index]:
            raise RuntimeError("nominal leg length is not feasible in both closed chains")

        first = nominal_index
        while first > 0 and feasible[first - 1]:
            first -= 1
        last = nominal_index
        while last + 1 < len(lengths) and feasible[last + 1]:
            last += 1
        if last - first < 2:
            raise RuntimeError("closed-chain safe leg-length interval is too small")
        return first, last

    @staticmethod
    def _quaternion_matrix(quaternion: np.ndarray) -> np.ndarray:
        matrix = np.zeros(9)
        mujoco.mju_quat2Mat(matrix, quaternion)
        return matrix.reshape(3, 3)

    def _world_inertia(self, body: int) -> np.ndarray:
        orientation = np.zeros(4)
        mujoco.mju_mulQuat(
            orientation, self.data.xquat[body], self.model.body_iquat[body])
        rotation = self._quaternion_matrix(orientation)
        return rotation @ np.diag(self.model.body_inertia[body]) @ rotation.T

    def _set_pose(
        self,
        solutions: tuple[PoseSolution, PoseSolution],
    ) -> None:
        for spec, solution in zip(SIDE_SPECS, solutions):
            addresses = self._side_addresses(spec)
            self.data.qpos[addresses["qpos"]] = solution.joint_position
        mujoco.mj_forward(self.model, self.data)

    def equivalent_leg(self, spec: SideSpec) -> dict[str, float]:
        body_ids = [
            self.require_id(mujoco.mjtObj.mjOBJ_BODY, name)
            for name in spec.leg_bodies
        ]
        masses = np.asarray([self.model.body_mass[body] for body in body_ids])
        total_mass = float(np.sum(masses))
        center = np.sum(
            masses[:, None] * self.data.xipos[body_ids], axis=0) / total_mass

        hip = self.data.site_xpos[
            self.require_id(mujoco.mjtObj.mjOBJ_SITE, spec.hip_site)]
        wheel = self.data.site_xpos[
            self.require_id(mujoco.mjtObj.mjOBJ_SITE, spec.wheel_site)]
        base = self.require_id(mujoco.mjtObj.mjOBJ_BODY, "base_link")
        base_rotation = self.data.xmat[base].reshape(3, 3)
        pitch_axis = base_rotation @ np.asarray((0.0, 1.0, 0.0))
        leg_axis = wheel - hip
        leg_axis -= pitch_axis * np.dot(leg_axis, pitch_axis)
        leg_axis /= np.linalg.norm(leg_axis)
        offset_axis = np.cross(pitch_axis, leg_axis)

        center_from_hip = center - hip
        com_distance = float(np.dot(center_from_hip, leg_axis))
        perpendicular_offset = float(np.dot(center_from_hip, offset_axis))
        inertia_about_hip = 0.0
        for body, mass in zip(body_ids, masses):
            displacement = self.data.xipos[body] - hip
            distance_squared = (
                np.dot(displacement, displacement)
                - np.dot(displacement, pitch_axis) ** 2)
            inertia_about_hip += (
                float(pitch_axis @ self._world_inertia(body) @ pitch_axis)
                + float(mass) * float(distance_squared)
            )
        equivalent_inertia = inertia_about_hip - total_mass * com_distance**2
        if equivalent_inertia <= 0.0:
            raise RuntimeError(f"non-positive equivalent inertia for {spec.name} leg")
        return {
            "mass": total_mass,
            "com_distance": com_distance,
            "com_perpendicular_offset": perpendicular_offset,
            "inertia": equivalent_inertia,
            "inertia_about_hip": inertia_about_hip,
        }

    def _wheel_radius(self, spec: SideSpec) -> float:
        geom = self.require_id(
            mujoco.mjtObj.mjOBJ_GEOM, spec.wheel_collision)
        mesh = self.model.geom_dataid[geom]
        start = self.model.mesh_vertadr[mesh]
        count = self.model.mesh_vertnum[mesh]
        vertices = self.model.mesh_vert[start:start + count]
        rotation = self.data.geom_xmat[geom].reshape(3, 3)
        world_vertices = self.data.geom_xpos[geom] + vertices @ rotation.T

        joint = self.require_id(mujoco.mjtObj.mjOBJ_JOINT, spec.wheel_joint)
        anchor = self.data.xanchor[joint]
        axis = self.data.xaxis[joint]
        displacement = world_vertices - anchor
        radial = displacement - np.outer(displacement @ axis, axis)
        return float(np.max(np.linalg.norm(radial, axis=1)))

    def _wheel_inertia(self, spec: SideSpec) -> tuple[float, float]:
        body = self.require_id(mujoco.mjtObj.mjOBJ_BODY, spec.wheel_body)
        joint = self.require_id(mujoco.mjtObj.mjOBJ_JOINT, spec.wheel_joint)
        axis = self.data.xaxis[joint]
        displacement = self.data.xipos[body] - self.data.xanchor[joint]
        radial_squared = (
            np.dot(displacement, displacement)
            - np.dot(displacement, axis) ** 2)
        inertia = (
            float(axis @ self._world_inertia(body) @ axis)
            + float(self.model.body_mass[body]) * float(radial_squared)
        )
        return float(self.model.body_mass[body]), inertia

    def rigid_parameters(self) -> dict[str, object]:
        base = self.require_id(mujoco.mjtObj.mjOBJ_BODY, "base_link")
        base_rotation = self.data.xmat[base].reshape(3, 3)
        pitch_axis = base_rotation @ np.asarray((0.0, 1.0, 0.0))
        yaw_axis = base_rotation @ np.asarray((0.0, 0.0, 1.0))
        base_inertia = self._world_inertia(base)

        hip_positions = np.asarray([
            self.data.site_xpos[
                self.require_id(mujoco.mjtObj.mjOBJ_SITE, spec.hip_site)]
            for spec in SIDE_SPECS
        ])
        wheel_positions = np.asarray([
            self.data.site_xpos[
                self.require_id(mujoco.mjtObj.mjOBJ_SITE, spec.wheel_site)]
            for spec in SIDE_SPECS
        ])
        hip_center = np.mean(hip_positions, axis=0)
        base_com_offset = self.data.xipos[base] - hip_center

        wheel_values = [self._wheel_inertia(spec) for spec in SIDE_SPECS]
        wheel_radii = [self._wheel_radius(spec) for spec in SIDE_SPECS]
        half_track = 0.5 * abs(wheel_positions[0, 1] - wheel_positions[1, 1])

        masses = self.model.body_mass
        total_mass = float(np.sum(masses))
        center = np.sum(masses[:, None] * self.data.xipos, axis=0) / total_mass
        assembly_yaw_inertia = 0.0
        for body in range(1, self.model.nbody):
            displacement = self.data.xipos[body] - center
            radial_squared = (
                np.dot(displacement, displacement)
                - np.dot(displacement, yaw_axis) ** 2)
            assembly_yaw_inertia += (
                float(yaw_axis @ self._world_inertia(body) @ yaw_axis)
                + float(masses[body]) * float(radial_squared)
            )

        return {
            "body_mass": float(self.model.body_mass[base]),
            "body_pitch_inertia": float(pitch_axis @ base_inertia @ pitch_axis),
            "body_yaw_inertia_actual": float(yaw_axis @ base_inertia @ yaw_axis),
            "body_com_height": float(np.dot(base_com_offset, yaw_axis)),
            "body_com_forward_offset": float(
                np.dot(base_com_offset, base_rotation[:, 0])),
            "wheel_mass": float(np.mean([value[0] for value in wheel_values])),
            "wheel_mass_spread": float(np.ptp([value[0] for value in wheel_values])),
            "wheel_inertia": float(np.mean([value[1] for value in wheel_values])),
            "wheel_inertia_spread": float(np.ptp([value[1] for value in wheel_values])),
            "wheel_radius": float(np.mean(wheel_radii)),
            "wheel_radius_spread": float(np.ptp(wheel_radii)),
            "half_track": float(half_track),
            "total_mass_diagnostic": total_mass,
            "assembly_yaw_inertia_diagnostic": float(assembly_yaw_inertia),
        }

    def extract(
        self,
        scan_minimum: float = 0.15,
        scan_maximum: float = 0.43,
        scan_step: float = 0.001,
        nominal_length: float = 0.34,
        sample_count: int = 31,
    ) -> dict[str, object]:
        count = int(round((scan_maximum - scan_minimum) / scan_step)) + 1
        lengths = np.linspace(scan_minimum, scan_maximum, count)
        side_scans = tuple(
            self.scan_side(spec, lengths, nominal_length)
            for spec in SIDE_SPECS
        )
        first, last = self.safe_range(lengths, side_scans, nominal_length)
        safe_minimum = float(lengths[first])
        safe_maximum = float(lengths[last])

        sample_lengths = np.linspace(safe_minimum, safe_maximum, sample_count)
        sampled_legs = {spec.name: [] for spec in SIDE_SPECS}
        maximum_closure_error = 0.0
        maximum_target_error = 0.0
        minimum_joint_margin = float("inf")
        nominal_solutions = None

        for length in sample_lengths:
            scan_index = int(np.argmin(np.abs(lengths - length)))
            initial = tuple(
                side_scans[side][scan_index].joint_position
                for side in range(len(SIDE_SPECS))
            )
            solutions = tuple(
                self.solve_pose(
                    spec, float(length), initial[side],
                    self.closure_limits[spec.name])
                for side, spec in enumerate(SIDE_SPECS)
            )
            if not all(solution.feasible for solution in solutions):
                raise RuntimeError(
                    f"resampled closed-chain pose is invalid at L={length:.6f}")
            self._set_pose(solutions)

            for spec, solution in zip(SIDE_SPECS, solutions):
                values = self.equivalent_leg(spec)
                values["length"] = float(length)
                sampled_legs[spec.name].append(values)
                maximum_closure_error = max(
                    maximum_closure_error, solution.closure_error)
                maximum_target_error = max(
                    maximum_target_error, solution.target_error)
                minimum_joint_margin = min(
                    minimum_joint_margin, solution.joint_margin)
            if nominal_solutions is None or abs(length - nominal_length) < abs(
                nominal_solutions[0].length - nominal_length):
                nominal_solutions = solutions

        if nominal_solutions is None:
            raise RuntimeError("could not resolve nominal closed-chain pose")
        self._set_pose(nominal_solutions)
        rigid = self.rigid_parameters()

        left_mass = sampled_legs["left"][0]["mass"]
        right_mass = sampled_legs["right"][0]["mass"]
        if abs(left_mass - right_mass) > 1.0e-9:
            raise RuntimeError("left and right reduced leg masses differ")
        represented_mass = (
            rigid["body_mass"] + 2.0 * left_mass
            + 2.0 * rigid["wheel_mass"])
        mass_error = abs(represented_mass - rigid["total_mass_diagnostic"])
        if mass_error > 1.0e-9:
            raise RuntimeError("reduced model body partition does not conserve mass")

        fit_diagnostics = {}
        for spec in SIDE_SPECS:
            values = sampled_legs[spec.name]
            sample_l = np.asarray([value["length"] for value in values])
            sample_d = np.asarray([value["com_distance"] for value in values])
            sample_i = np.asarray([value["inertia"] for value in values])
            com_fit = np.polyfit(sample_l, sample_d, 1)
            inertia_fit = np.polyfit(sample_l, sample_i, 2)
            fit_diagnostics[spec.name] = {
                "com_affine_coefficients": com_fit.tolist(),
                "com_maximum_error": float(np.max(np.abs(
                    np.polyval(com_fit, sample_l) - sample_d))),
                "inertia_quadratic_coefficients": inertia_fit.tolist(),
                "inertia_maximum_error": float(np.max(np.abs(
                    np.polyval(inertia_fit, sample_l) - sample_i))),
                "maximum_com_perpendicular_offset": float(np.max(np.abs([
                    value["com_perpendicular_offset"] for value in values
                ]))),
            }

        return {
            "model_path": str(self.model_path),
            "safe_length_range": [safe_minimum, safe_maximum],
            "scan": {
                "minimum": scan_minimum,
                "maximum": scan_maximum,
                "step": scan_step,
                "nominal_length": nominal_length,
                "maximum_closure_error": maximum_closure_error,
                "maximum_target_error": maximum_target_error,
                "minimum_joint_margin": minimum_joint_margin,
                "closure_limits": self.closure_limits,
            },
            "rigid": rigid,
            "leg_mass": float(left_mass),
            "mass_partition_error": float(mass_error),
            "legs": sampled_legs,
            "fit_diagnostics": fit_diagnostics,
        }
