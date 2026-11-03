//! Predict the moving target and return an aiming command in [`aim`].
//! World axes: +X forward, +Y left, +Z up. Distances are metres and angles are radians.
//! The simulator calls `aim` every 10 ms with a perfect observation captured in the past.

pub mod sim;
pub mod ui;

/// Fixed launcher physics and timing shared by the controller, viewer, and evaluator.
pub mod physics {
    /// Stationary muzzle position in world metres.
    pub const MUZZLE_M: [f64; 3] = [0.0, 0.0, 0.3];
    /// Projectile launch speed in metres per second.
    pub const MUZZLE_SPEED_M_S: f64 = 25.0;
    /// Downward acceleration in metres per second squared.
    pub const GRAVITY_M_S2: f64 = 9.81;
    /// Time from observation capture to delivery.
    pub const OBSERVATION_DELAY_S: f64 = 0.05;
    /// Time from command acceptance to projectile launch.
    pub const LAUNCH_DELAY_S: f64 = 0.02;
    /// Minimum interval between accepted shots.
    pub const FIRE_PERIOD_S: f64 = 0.25;
    /// Projectile lifetime after launch.
    pub const MAX_FLIGHT_S: f64 = 2.0;
    /// Fixed interval between controller calls.
    pub const CONTROL_PERIOD_S: f64 = 0.01;
}

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::f64::consts::TAU;
use ui::{Color, Gizmos};

/// Predict one plate at impact, then aim and gate firing on relative incidence.
pub fn aim(input: &Input, debug: &mut Gizmos) -> Option<Command> {
    let resolved = input.stats.hits.saturating_add(input.stats.misses);
    debug.overlay("Hit rate", format!("{}/{resolved}", input.stats.hits));

    let Some(observation) = input.observation else {
        debug.overlay("Status", "Waiting for observation");
        return None;
    };
    let age_s = input.now_s - observation.time_s;
    if !valid_observation(input, observation) {
        debug.overlay("Status", "Invalid observation or geometry");
        return None;
    }
    for plate in observation.target.plates(input.size) {
        debug.plate(plate, input.size, Color::Cyan);
    }

    let Some(solution) = intercept(observation.target, input.size, age_s) else {
        debug.overlay("Status", "No reachable, converged intercept");
        return None;
    };
    let fire = solution.incidence_cos >= MIN_INCIDENCE_COS;
    let trajectory = solution.trajectory;
    let impact = trajectory.position(trajectory.flight_time_s);
    debug.plate(solution.plate, input.size, Color::Green);
    debug.trajectory(trajectory, Color::Orange);
    debug.point("Predicted center", solution.plate.center_m, Color::Green);
    debug.point("Impact", impact, Color::Orange);
    debug.line(solution.plate.center_m, impact, Color::Purple);
    debug.overlay("Status", if fire { "Fire" } else { "Track: oblique entry" });
    debug.overlay("Plate", solution.slot);
    debug.overlay("Observation age", format!("{:.1} ms", age_s * 1000.0));
    debug.overlay(
        "Flight",
        format!("{:.1} ms", trajectory.flight_time_s * 1000.0),
    );
    // Absolute impact time starts at now; the observation age must not be added twice.
    debug.overlay(
        "Impact time",
        format!(
            "{:.4} s",
            input.now_s + physics::LAUNCH_DELAY_S + trajectory.flight_time_s
        ),
    );
    debug.overlay("Incidence cosine", format!("{:.4}", solution.incidence_cos));
    debug.overlay("Iterations", solution.iterations);
    debug.overlay(
        "Endpoint residual",
        format!("{:.6} mm", solution.residual_m * 1000.0),
    );
    Some(Command {
        yaw_rad: trajectory.yaw_rad,
        pitch_rad: trajectory.pitch_rad,
        fire,
    })
}

/// Require entry within 60 degrees of the full 3D outward normal.
const MIN_INCIDENCE_COS: f64 = 0.5;
const INTERCEPT_TIME_TOLERANCE_S: f64 = 1e-7;
const MAX_INTERCEPT_ITERATIONS: usize = 48;
const MAX_ENDPOINT_RESIDUAL_M: f64 = 1e-4;

#[derive(Clone, Copy, Debug)]
struct Intercept {
    slot: usize,
    plate: Plate,
    trajectory: Trajectory,
    incidence_cos: f64,
    iterations: usize,
    residual_m: f64,
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a.into_iter().zip(b).map(|(x, y)| x * y).sum()
}

fn difference(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|axis| a[axis] - b[axis])
}

fn length(vector: [f64; 3]) -> f64 {
    vector[0].hypot(vector[1]).hypot(vector[2])
}

fn valid_observation(input: &Input, observation: Sample) -> bool {
    let target = observation.target;
    let size = input.size;
    [
        input.now_s,
        observation.time_s,
        target.x_m,
        target.y_m,
        target.theta_rad,
        target.vx_m_s,
        target.vy_m_s,
        target.omega_rad_s,
        size.mount_pitch_rad,
        size.radius_m,
        size.plate_width_m,
        size.plate_height_m,
        size.plate_center_z_m,
    ]
    .into_iter()
    .all(f64::is_finite)
        && observation.time_s >= 0.0
        && input.now_s >= observation.time_s
        && size.radius_m > 0.0
        && size.plate_width_m > 0.0
        && size.plate_height_m > 0.0
        && size.plate_center_z_m > 0.0
}

fn intercept(target: Target, size: Size, age_s: f64) -> Option<Intercept> {
    let delay_s = age_s + physics::LAUNCH_DELAY_S;
    let at_launch = target.at(delay_s);
    let center = [at_launch.x_m, at_launch.y_m, size.plate_center_z_m];
    let mut flight_s = length(difference(center, physics::MUZZLE_M)) / physics::MUZZLE_SPEED_M_S;
    if !flight_s.is_finite() || flight_s <= 0.0 {
        return None;
    }

    // Select once using a cheap arrival estimate; never solve all four trajectories
    // or change plate identity inside the fixed-point iteration.
    let mut selected = None;
    let mut best_facing = 0.0;
    for (slot, plate) in target
        .at(delay_s + flight_s)
        .plates(size)
        .into_iter()
        .enumerate()
    {
        let to_muzzle = difference(physics::MUZZLE_M, plate.center_m);
        let distance = length(to_muzzle);
        if distance > 0.0 {
            let facing = dot(plate.outward_normal, to_muzzle) / distance;
            if facing > best_facing {
                best_facing = facing;
                selected = Some(slot);
            }
        }
    }
    let slot = selected?;
    for iterations in 1..=MAX_INTERCEPT_ITERATIONS {
        let plate = target.at(delay_s + flight_s).plates(size)[slot];
        let trajectory = Trajectory::solve(plate.center_m)?;
        let next_flight_s = trajectory.flight_time_s;
        if (next_flight_s - flight_s).abs() <= INTERCEPT_TIME_TOLERANCE_S {
            // Check consistency at the returned flight time, not the preceding guess.
            let plate = target.at(delay_s + next_flight_s).plates(size)[slot];
            let residual_m = length(difference(
                trajectory.position(next_flight_s),
                plate.center_m,
            ));
            let relative_velocity =
                difference(trajectory.velocity(next_flight_s), plate.velocity_m_s);
            let relative_speed = length(relative_velocity);
            if !residual_m.is_finite()
                || residual_m > MAX_ENDPOINT_RESIDUAL_M
                || !relative_speed.is_finite()
                || relative_speed <= 0.0
            {
                return None;
            }
            let incidence_cos = -dot(plate.outward_normal, relative_velocity) / relative_speed;
            if !incidence_cos.is_finite() {
                return None;
            }
            return Some(Intercept {
                slot,
                plate,
                trajectory,
                incidence_cos,
                iterations,
                residual_m,
            });
        }
        // Relaxation damps oscillation for fast spin while preserving the fixed point.
        flight_s = 0.5 * (flight_s + next_flight_s);
    }
    None
}

/// The only simulation data provided to the applicant's controller.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Input {
    pub now_s: f64,
    /// None until the first observation's configured delivery delay has elapsed.
    pub observation: Option<Sample>,
    pub size: Size,
    /// Outcomes resolved by this tick, before the current command is accepted.
    #[serde(default)]
    pub stats: Stats,
}

/// World yaw, upward pitch, and a request to fire at the fixed maximum rate.
/// Angles are latched when a shot is accepted, then applied after `physics::LAUNCH_DELAY_S`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Command {
    pub yaw_rad: f64,
    pub pitch_rad: f64,
    pub fire: bool,
}

/// A gravity-only projectile path. Flight time starts at launch, excluding all delays.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Trajectory {
    pub yaw_rad: f64,
    pub pitch_rad: f64,
    pub flight_time_s: f64,
}

impl Trajectory {
    /// Solve the low arc to a fixed world point at the fixed muzzle speed.
    /// Returns `None` for an unreachable point, nonfinite input, zero flight time, or timeout.
    /// Predict the moving target separately; this method does not compensate for delays.
    pub fn solve(point_m: [f64; 3]) -> Option<Self> {
        let delta: [f64; 3] = std::array::from_fn(|i| point_m[i] - physics::MUZZLE_M[i]);
        let distance_squared = delta.iter().map(|v| v * v).sum::<f64>();
        let gravity = physics::GRAVITY_M_S2;
        let a = physics::MUZZLE_SPEED_M_S.powi(2) - gravity * delta[2];
        let discriminant = a * a - gravity * gravity * distance_squared;
        if a <= 0.0 || discriminant < 0.0 {
            return None;
        }
        // Rationalized smaller root of the ballistic equation in flight time squared.
        let flight_time_s = (2.0 * distance_squared / (a + discriminant.sqrt())).sqrt();
        if !flight_time_s.is_finite()
            || flight_time_s <= 0.0
            || flight_time_s > physics::MAX_FLIGHT_S
        {
            return None;
        }
        let mut velocity = delta.map(|d| d / flight_time_s);
        velocity[2] += 0.5 * gravity * flight_time_s;
        Some(Self {
            yaw_rad: velocity[1].atan2(velocity[0]),
            pitch_rad: velocity[2].atan2(velocity[0].hypot(velocity[1])),
            flight_time_s,
        })
    }

    /// World position `elapsed_s` after launch; gravity produces a mathematical parabola.
    pub fn position(self, elapsed_s: f64) -> [f64; 3] {
        let velocity = self.velocity(0.0);
        std::array::from_fn(|i| {
            physics::MUZZLE_M[i] + velocity[i] * elapsed_s
                - if i == 2 {
                    0.5 * physics::GRAVITY_M_S2 * elapsed_s * elapsed_s
                } else {
                    0.0
                }
        })
    }

    /// World velocity `elapsed_s` after launch, before subtracting the armor's velocity.
    pub fn velocity(self, elapsed_s: f64) -> [f64; 3] {
        let horizontal = physics::MUZZLE_SPEED_M_S * self.pitch_rad.cos();
        [
            horizontal * self.yaw_rad.cos(),
            horizontal * self.yaw_rad.sin(),
            physics::MUZZLE_SPEED_M_S * self.pitch_rad.sin() - physics::GRAVITY_M_S2 * elapsed_s,
        ]
    }
}

/// World axes are +X forward, +Y left, +Z up. Angles turn from +X toward +Y.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Target {
    pub x_m: f64,
    pub y_m: f64,
    pub theta_rad: f64,
    pub vx_m_s: f64,
    pub vy_m_s: f64,
    pub omega_rad_s: f64,
}

impl Target {
    /// Evaluate constant world velocity and spin without integrating intermediate steps.
    pub fn at(self, elapsed_s: f64) -> Self {
        Self {
            x_m: self.x_m + self.vx_m_s * elapsed_s,
            y_m: self.y_m + self.vy_m_s * elapsed_s,
            theta_rad: (self.theta_rad + self.omega_rad_s * elapsed_s).rem_euclid(TAU),
            ..self
        }
    }

    /// Four equally spaced plates, mounted with their top edges tilted toward the center.
    pub fn plates(self, size: Size) -> [Plate; 4] {
        std::array::from_fn(|slot| {
            let angle = self.theta_rad + slot as f64 * TAU / 4.0;
            let (sin, cos) = angle.sin_cos();
            let (tilt_sin, tilt_cos) = size.mount_pitch_rad.sin_cos();
            Plate {
                center_m: [
                    self.x_m + size.radius_m * cos,
                    self.y_m + size.radius_m * sin,
                    size.plate_center_z_m,
                ],
                outward_normal: [cos * tilt_cos, sin * tilt_cos, tilt_sin],
                right_axis: [-sin, cos, 0.0],
                up_axis: [-cos * tilt_sin, -sin * tilt_sin, tilt_cos],
                velocity_m_s: [
                    self.vx_m_s - self.omega_rad_s * size.radius_m * sin,
                    self.vy_m_s + self.omega_rad_s * size.radius_m * cos,
                    0.0,
                ],
            }
        })
    }
}

/// Scoring rectangle and mounting geometry. Plates have equal radii and heights.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(default, deny_unknown_fields)]
pub struct Size {
    /// Positive pitch tilts the outward normal up, matching the main simulator.
    pub mount_pitch_rad: f64,
    pub radius_m: f64,
    pub plate_width_m: f64,
    pub plate_height_m: f64,
    pub plate_center_z_m: f64,
}

/// Ground-truth plate state; plate indices remain attached to the target as it turns.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Plate {
    pub center_m: [f64; 3],
    pub outward_normal: [f64; 3],
    pub right_axis: [f64; 3],
    pub up_axis: [f64; 3],
    pub velocity_m_s: [f64; 3],
}

impl Plate {
    /// Convert plate coordinates (right, up, outward), in metres, to world coordinates.
    pub fn point(self, right_m: f64, up_m: f64, outward_m: f64) -> [f64; 3] {
        std::array::from_fn(|i| {
            self.center_m[i]
                + self.right_axis[i] * right_m
                + self.up_axis[i] * up_m
                + self.outward_normal[i] * outward_m
        })
    }
}

/// Perfect target state at the original observation capture time.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Sample {
    pub time_s: f64,
    pub target: Target,
}

/// Accepted shots and their outcomes. Requests blocked by the firing rate are not shots.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Stats {
    pub shots: usize,
    pub hits: usize,
    pub misses: usize,
    pub invalid_commands: usize,
}
