#!/usr/bin/env python3
"""MATLAB-free gain-scheduled LQR generator for the balance chassis."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np
import sympy as sp
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete


STATE_NAMES = (
    "s", "ds", "psi", "dpsi", "theta_l", "dtheta_l",
    "theta_r", "dtheta_r", "theta_b", "dtheta_b",
)
INPUT_NAMES = ("T_wheel_l", "T_wheel_r", "Tp_leg_l", "Tp_leg_r")


@dataclass(frozen=True)
class PhysicalParameters:
    wheel_radius: float
    half_track: float
    body_com_height: float
    wheel_mass: float
    leg_mass: float
    body_mass: float
    gravity: float
    wheel_inertia: float
    body_pitch_inertia: float
    body_yaw_inertia_actual: float
    body_yaw_inertia_scale: float = 1.0

    @property
    def body_yaw_inertia_model(self) -> float:
        return self.body_yaw_inertia_actual * self.body_yaw_inertia_scale


@dataclass(frozen=True)
class LqrSettings:
    length_min: float
    length_max: float
    sample_count: int
    timestep: float
    q_diagonal: tuple[float, ...]
    r_diagonal: tuple[float, ...]


@dataclass(frozen=True)
class LegParameters:
    left_com_distance: float
    right_com_distance: float
    left_inertia: float
    right_inertia: float


@dataclass
class GainSamples:
    lengths: np.ndarray
    gains: np.ndarray
    maximum_eigenvalue: float
    maximum_are_residual: float
    minimum_controllability_rank: int


@dataclass
class GainSchedule:
    coefficient: np.ndarray
    polynomial_order: int
    length_midpoint: float
    length_scale: float
    maximum_fit_error: float
    maximum_dense_eigenvalue: float


def legacy_parameters() -> PhysicalParameters:
    wheel_mass = 0.710
    wheel_radius = 0.068
    return PhysicalParameters(
        wheel_radius=wheel_radius,
        half_track=0.214,
        body_com_height=0.013,
        wheel_mass=wheel_mass,
        leg_mass=1.190,
        body_mass=17.650,
        gravity=9.807,
        wheel_inertia=wheel_mass * wheel_radius**2 / 2.0,
        body_pitch_inertia=367565e-6,
        body_yaw_inertia_actual=413477e-6,
    )


def legacy_settings() -> LqrSettings:
    return LqrSettings(
        length_min=0.15,
        length_max=0.40,
        sample_count=31,
        timestep=0.003,
        q_diagonal=(180, 60, 40, 15, 30, 0.8, 30, 0.8, 800, 120),
        r_diagonal=(3.2, 3.2, 0.7, 0.7),
    )


def legacy_leg_parameters(length: float) -> LegParameters:
    inertia_min = 13000e-6
    inertia_max = 15800e-6
    ratio = (length - 0.15) / (0.40 - 0.15)
    inertia = inertia_min + (inertia_max - inertia_min) * ratio
    com_distance = 0.28 * length
    return LegParameters(com_distance, com_distance, inertia, inertia)


def verify_yaw_inertia_identity() -> bool:
    """Prove that equ5 contains the complete yaw inertia, not half of it."""
    inertia, wheel_radius, half_track = sp.symbols(
        "I_z R_w R_l", nonzero=True, real=True)
    length_l, length_r = sp.symbols("L_l L_r", real=True)
    wheel_l, wheel_r, leg_l, leg_r, yaw = sp.symbols(
        "a_wl a_wr a_ll a_lr a_psi", real=True)

    inertia_terms = inertia / (2 * half_track) * (
        wheel_radius * wheel_l - wheel_radius * wheel_r
        + length_l * leg_l - length_r * leg_r)
    yaw_definition = sp.Eq(
        yaw,
        (-wheel_radius * wheel_l + wheel_radius * wheel_r
         - length_l * leg_l + length_r * leg_r) / (2 * half_track),
    )
    solved_wheel_l = sp.solve(yaw_definition, wheel_l)[0]
    reduced = sp.simplify(inertia_terms.subs(wheel_l, solved_wheel_l))
    return sp.simplify(reduced + inertia * yaw) == 0


def build_state_matrix_functions(
    physical: PhysicalParameters,
) -> tuple[Callable[..., np.ndarray], Callable[..., np.ndarray]]:
    """Build numerical A/B functions with the equations from lqr.m."""
    acceleration_s, acceleration_yaw = sp.symbols("D2s D2psi", real=True)
    acceleration_wheel_l, acceleration_wheel_r = sp.symbols(
        "D2theta_wl D2theta_wr", real=True)
    acceleration_leg_l, acceleration_leg_r = sp.symbols(
        "D2theta_ll D2theta_lr", real=True)
    acceleration_body = sp.symbols("D2theta_b", real=True)

    s, velocity_s, yaw, velocity_yaw = sp.symbols(
        "s Ds psi Dpsi", real=True)
    leg_l, velocity_leg_l = sp.symbols("theta_ll Dtheta_ll", real=True)
    leg_r, velocity_leg_r = sp.symbols("theta_lr Dtheta_lr", real=True)
    body, velocity_body = sp.symbols("theta_b Dtheta_b", real=True)
    torque_wheel_l, torque_wheel_r, torque_leg_l, torque_leg_r = sp.symbols(
        "T_lwl T_lwr T_bll T_blr", real=True)

    length_l, length_r, com_l, com_r, inertia_l, inertia_r = sp.symbols(
        "L_l L_r L_bl L_br I_ll I_lr", real=True)
    wheel_to_com_l = length_l - com_l
    wheel_to_com_r = length_r - com_r

    rw = physical.wheel_radius
    rl = physical.half_track
    lc = physical.body_com_height
    mw = physical.wheel_mass
    ml = physical.leg_mass
    mb = physical.body_mass
    gravity = physical.gravity
    iw = physical.wheel_inertia
    ib = physical.body_pitch_inertia
    iz = physical.body_yaw_inertia_model

    equations = [
        sp.Eq(
            (iw * length_l / rw + mw * rw * length_l + ml * rw * com_l)
            * acceleration_wheel_l
            + (ml * wheel_to_com_l * com_l - inertia_l) * acceleration_leg_l
            + (ml * wheel_to_com_l + mb * length_l / 2) * gravity * leg_l
            + torque_leg_l - torque_wheel_l * (1 + length_l / rw),
            0,
        ),
        sp.Eq(
            (iw * length_r / rw + mw * rw * length_r + ml * rw * com_r)
            * acceleration_wheel_r
            + (ml * wheel_to_com_r * com_r - inertia_r) * acceleration_leg_r
            + (ml * wheel_to_com_r + mb * length_r / 2) * gravity * leg_r
            + torque_leg_r - torque_wheel_r * (1 + length_r / rw),
            0,
        ),
        sp.Eq(
            -(mw * rw**2 + iw + ml * rw**2 + mb * rw**2 / 2)
            * acceleration_wheel_l
            - (mw * rw**2 + iw + ml * rw**2 + mb * rw**2 / 2)
            * acceleration_wheel_r
            - (ml * rw * wheel_to_com_l + mb * rw * length_l / 2)
            * acceleration_leg_l
            - (ml * rw * wheel_to_com_r + mb * rw * length_r / 2)
            * acceleration_leg_r
            + torque_wheel_l + torque_wheel_r,
            0,
        ),
        sp.Eq(
            (mw * rw * lc + iw * lc / rw + ml * rw * lc)
            * (acceleration_wheel_l + acceleration_wheel_r)
            + ml * wheel_to_com_l * lc * acceleration_leg_l
            + ml * wheel_to_com_r * lc * acceleration_leg_r
            - ib * acceleration_body + mb * gravity * lc * body
            - (torque_wheel_l + torque_wheel_r) * lc / rw
            - torque_leg_l - torque_leg_r,
            0,
        ),
        sp.Eq(
            (iz * rw / (2 * rl) + iw * rl / rw) * acceleration_wheel_l
            - (iz * rw / (2 * rl) + iw * rl / rw) * acceleration_wheel_r
            + iz * length_l * acceleration_leg_l / (2 * rl)
            - iz * length_r * acceleration_leg_r / (2 * rl)
            - torque_wheel_l * rl / rw + torque_wheel_r * rl / rw,
            0,
        ),
        sp.Eq(
            acceleration_s,
            rw * (acceleration_wheel_l + acceleration_wheel_r) / 2,
        ),
        sp.Eq(
            acceleration_yaw,
            rw * (-acceleration_wheel_l + acceleration_wheel_r) / (2 * rl)
            - length_l * acceleration_leg_l / (2 * rl)
            + length_r * acceleration_leg_r / (2 * rl),
        ),
    ]
    unknowns = (
        acceleration_s, acceleration_wheel_l, acceleration_wheel_r,
        acceleration_leg_l, acceleration_leg_r, acceleration_body,
        acceleration_yaw,
    )
    solution = sp.solve(equations, unknowns, dict=True)[0]

    state = (
        s, velocity_s, yaw, velocity_yaw, leg_l, velocity_leg_l,
        leg_r, velocity_leg_r, body, velocity_body,
    )
    control = (torque_wheel_l, torque_wheel_r, torque_leg_l, torque_leg_r)
    state_derivative = sp.Matrix((
        velocity_s, solution[acceleration_s],
        velocity_yaw, solution[acceleration_yaw],
        velocity_leg_l, solution[acceleration_leg_l],
        velocity_leg_r, solution[acceleration_leg_r],
        velocity_body, solution[acceleration_body],
    ))
    sample_parameters = (
        length_l, length_r, com_l, com_r, inertia_l, inertia_r,
    )
    matrix_a = sp.lambdify(
        sample_parameters, state_derivative.jacobian(state), "numpy")
    matrix_b = sp.lambdify(
        sample_parameters, state_derivative.jacobian(control), "numpy")
    return matrix_a, matrix_b


def evaluate_state_matrices(
    functions: tuple[Callable[..., np.ndarray], Callable[..., np.ndarray]],
    length: float,
    leg: LegParameters,
) -> tuple[np.ndarray, np.ndarray]:
    arguments = (
        length, length, leg.left_com_distance, leg.right_com_distance,
        leg.left_inertia, leg.right_inertia,
    )
    return (
        np.asarray(functions[0](*arguments), dtype=float),
        np.asarray(functions[1](*arguments), dtype=float),
    )


def discretize(
    matrix_a: np.ndarray,
    matrix_b: np.ndarray,
    timestep: float,
) -> tuple[np.ndarray, np.ndarray]:
    outputs = np.eye(len(STATE_NAMES))
    feedthrough = np.zeros((len(STATE_NAMES), len(INPUT_NAMES)))
    discrete = cont2discrete(
        (matrix_a, matrix_b, outputs, feedthrough), timestep, method="zoh")
    return discrete[0], discrete[1]


def solve_lqr(
    matrix_a: np.ndarray,
    matrix_b: np.ndarray,
    matrix_q: np.ndarray,
    matrix_r: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    riccati = solve_discrete_are(matrix_a, matrix_b, matrix_q, matrix_r)
    denominator = matrix_r + matrix_b.T @ riccati @ matrix_b
    gain = np.linalg.solve(
        denominator, matrix_b.T @ riccati @ matrix_a)
    eigenvalues = np.linalg.eigvals(matrix_a - matrix_b @ gain)
    residual = (
        riccati - matrix_a.T @ riccati @ matrix_a
        + matrix_a.T @ riccati @ matrix_b
        @ np.linalg.solve(denominator, matrix_b.T @ riccati @ matrix_a)
        - matrix_q
    )
    return gain, riccati, eigenvalues, float(np.max(np.abs(residual)))


def compute_gain_samples(
    physical: PhysicalParameters,
    settings: LqrSettings,
    leg_provider: Callable[[float], LegParameters],
) -> GainSamples:
    functions = build_state_matrix_functions(physical)
    lengths = np.linspace(
        settings.length_min, settings.length_max, settings.sample_count)
    gains = np.zeros((len(INPUT_NAMES), len(STATE_NAMES), len(lengths)))
    matrix_q = np.diag(settings.q_diagonal) * settings.timestep
    matrix_r = np.diag(settings.r_diagonal) * settings.timestep
    maximum_eigenvalue = 0.0
    maximum_residual = 0.0
    minimum_rank = len(STATE_NAMES)

    for index, length in enumerate(lengths):
        matrix_a, matrix_b = evaluate_state_matrices(
            functions, float(length), leg_provider(float(length)))
        discrete_a, discrete_b = discretize(
            matrix_a, matrix_b, settings.timestep)
        gain, _, eigenvalues, residual = solve_lqr(
            discrete_a, discrete_b, matrix_q, matrix_r)
        gains[:, :, index] = gain
        maximum_eigenvalue = max(
            maximum_eigenvalue, float(np.max(np.abs(eigenvalues))))
        maximum_residual = max(maximum_residual, residual)

        power = np.eye(len(STATE_NAMES))
        blocks = []
        for _ in range(len(STATE_NAMES)):
            blocks.append(power @ discrete_b)
            power = power @ discrete_a
        controllability = np.hstack(blocks)
        minimum_rank = min(minimum_rank, int(np.linalg.matrix_rank(controllability)))

    return GainSamples(
        lengths=lengths,
        gains=gains,
        maximum_eigenvalue=maximum_eigenvalue,
        maximum_are_residual=maximum_residual,
        minimum_controllability_rank=minimum_rank,
    )


def fit_gain_schedule(samples: GainSamples, polynomial_order: int) -> GainSchedule:
    length_midpoint = 0.5 * (samples.lengths[0] + samples.lengths[-1])
    length_scale = 0.5 * (samples.lengths[-1] - samples.lengths[0])
    normalized = (samples.lengths - length_midpoint) / length_scale
    coefficient = np.zeros(
        (len(INPUT_NAMES), len(STATE_NAMES), polynomial_order + 1))
    maximum_error = 0.0

    for input_index in range(len(INPUT_NAMES)):
        for state_index in range(len(STATE_NAMES)):
            values = samples.gains[input_index, state_index, :]
            fitted = np.polyfit(normalized, values, polynomial_order)
            coefficient[input_index, state_index, :] = fitted
            error = np.max(np.abs(np.polyval(fitted, normalized) - values))
            maximum_error = max(maximum_error, float(error))

    return GainSchedule(
        coefficient=coefficient,
        polynomial_order=polynomial_order,
        length_midpoint=length_midpoint,
        length_scale=length_scale,
        maximum_fit_error=maximum_error,
        maximum_dense_eigenvalue=float("inf"),
    )


def evaluate_gain(schedule: GainSchedule, length: float) -> np.ndarray:
    normalized = (
        length - schedule.length_midpoint) / schedule.length_scale
    gain = np.zeros((len(INPUT_NAMES), len(STATE_NAMES)))
    for input_index in range(len(INPUT_NAMES)):
        for state_index in range(len(STATE_NAMES)):
            gain[input_index, state_index] = np.polyval(
                schedule.coefficient[input_index, state_index, :], normalized)
    return gain


def validate_fitted_schedule(
    physical: PhysicalParameters,
    settings: LqrSettings,
    leg_provider: Callable[[float], LegParameters],
    schedule: GainSchedule,
    dense_count: int = 201,
) -> float:
    functions = build_state_matrix_functions(physical)
    maximum_eigenvalue = 0.0
    for length in np.linspace(
        settings.length_min, settings.length_max, dense_count):
        matrix_a, matrix_b = evaluate_state_matrices(
            functions, float(length), leg_provider(float(length)))
        discrete_a, discrete_b = discretize(
            matrix_a, matrix_b, settings.timestep)
        eigenvalues = np.linalg.eigvals(
            discrete_a - discrete_b @ evaluate_gain(schedule, float(length)))
        maximum_eigenvalue = max(
            maximum_eigenvalue, float(np.max(np.abs(eigenvalues))))
    schedule.maximum_dense_eigenvalue = maximum_eigenvalue
    return maximum_eigenvalue
