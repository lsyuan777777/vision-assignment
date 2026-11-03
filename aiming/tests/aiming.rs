//! Controller behavior tested against the unchanged, independent collision evaluator.
use aiming_sim::{
    Command, Input, Sample, Size, Stats, Target, aim, physics,
    sim::{Config, Frame, Simulation},
    ui::Gizmos,
};
use std::{error::Error, f64::consts::PI};

fn input_at(now_s: f64, capture_s: f64, initial: Target) -> Input {
    Input {
        now_s,
        observation: Some(Sample {
            time_s: capture_s,
            target: initial.at(capture_s),
        }),
        size: Size::default(),
        stats: Stats::default(),
    }
}

fn simulate(config: Config) -> Result<Frame, Box<dyn Error>> {
    let mut sim = Simulation::new(config)?;
    let mut controller = |input: &Input, debug: &mut Gizmos| {
        (input.now_s < config.duration_s)
            .then(|| aim(input, debug))
            .flatten()
    };
    let end_s = config.duration_s + physics::LAUNCH_DELAY_S + physics::MAX_FLIGHT_S;
    loop {
        let frame = sim.step(&mut controller);
        if frame.time_s >= end_s {
            assert_eq!(frame.stats.invalid_commands, 0);
            assert_eq!(
                frame.stats.hits + frame.stats.misses,
                frame.stats.shots,
                "all projectiles must be settled before measuring hit rate"
            );
            return Ok(frame);
        }
    }
}

#[test]
fn official_scenarios_hit_while_maintaining_firing_opportunities() -> Result<(), Box<dyn Error>> {
    for (name, json) in [
        ("stationary", include_str!("../scenarios/stationary.json")),
        ("spinning", include_str!("../scenarios/spinning.json")),
        ("moving", include_str!("../scenarios/moving.json")),
    ] {
        let config: Config = serde_json::from_str(json)?;
        let frame = simulate(config)?;
        assert!(
            frame.stats.shots >= 70,
            "{name}: insufficient shots: {:?}",
            frame.stats
        );
        assert_eq!(
            frame.stats.hits, frame.stats.shots,
            "{name}: {:?}",
            frame.stats
        );
    }
    Ok(())
}

#[test]
fn moving_targets_with_both_spin_directions_and_varied_phase() -> Result<(), Box<dyn Error>> {
    for omega in [-16.0, 0.0, 16.0] {
        for phase in [-PI, 0.37, PI / 3.0] {
            for (vx, vy) in [(0.0, 0.0), (0.4, -0.3), (-0.2, 0.35)] {
                let config = Config {
                    target: Target {
                        x_m: 5.0,
                        y_m: 0.6,
                        theta_rad: phase,
                        vx_m_s: vx,
                        vy_m_s: vy,
                        omega_rad_s: omega,
                    },
                    duration_s: 5.0,
                    ..Config::default()
                };
                let frame = simulate(config)?;
                assert!(
                    frame.stats.shots >= 12,
                    "too few shots for {config:?}: {:?}",
                    frame.stats
                );
                assert_eq!(frame.stats.misses, 0, "{config:?}: {:?}", frame.stats);
            }
        }
    }
    Ok(())
}

#[test]
fn observation_age_is_measured_and_not_assumed_or_double_counted() -> Result<(), Box<dyn Error>> {
    let target = Target {
        theta_rad: 0.37,
        vx_m_s: 0.6,
        vy_m_s: -0.4,
        omega_rad_s: -3.0,
        ..Target::default()
    };
    let recent =
        aim(&input_at(0.30, 0.25, target), &mut Gizmos::default()).ok_or("recent observation")?;
    let old = aim(&input_at(0.30, 0.00, target), &mut Gizmos::default())
        .ok_or("older exact observation")?;
    assert!((recent.yaw_rad - old.yaw_rad).abs() < 1e-10);
    assert!((recent.pitch_rad - old.pitch_rad).abs() < 1e-10);
    assert_eq!(recent.fire, old.fire);
    Ok(())
}

#[test]
fn stationary_aim_has_correct_yaw_and_gravity_compensation() -> Result<(), Box<dyn Error>> {
    let target = Target {
        omega_rad_s: 0.0,
        ..Target::default()
    };
    let input = input_at(1.0, 0.95, target);
    let command = aim(&input, &mut Gizmos::default()).ok_or("stationary target")?;
    let range = target.x_m - input.size.radius_m;
    let expected_pitch =
        0.5 * (physics::GRAVITY_M_S2 * range / physics::MUZZLE_SPEED_M_S.powi(2)).asin();
    assert!(command.fire);
    assert!(command.yaw_rad.abs() < 1e-12);
    assert!((command.pitch_rad - expected_pitch).abs() < 1e-12);
    Ok(())
}

#[test]
fn missing_future_nonfinite_and_invalid_geometry_inputs_cannot_fire() {
    let valid = input_at(0.3, 0.25, Target::default());
    let mut missing = valid;
    missing.observation = None;
    let mut future = valid;
    future.now_s = 0.1;
    let mut nonfinite = valid;
    nonfinite.now_s = f64::NAN;
    let mut invalid_target = valid;
    invalid_target.observation = Some(Sample {
        time_s: 0.25,
        target: Target {
            omega_rad_s: f64::INFINITY,
            ..Target::default()
        },
    });
    let mut invalid_size = valid;
    invalid_size.size.radius_m = 0.0;
    for input in [missing, future, nonfinite, invalid_target, invalid_size] {
        assert_eq!(aim(&input, &mut Gizmos::default()), None);
    }
}

#[test]
fn unreachable_and_inside_target_geometry_are_rejected() -> Result<(), Box<dyn Error>> {
    for x in [0.0, 60.0, 100.0] {
        let target = Target {
            x_m: x,
            omega_rad_s: 0.0,
            ..Target::default()
        };
        let input = input_at(0.3, 0.25, target);
        assert_eq!(aim(&input, &mut Gizmos::default()), None, "x={x}");
    }
    let frame = simulate(Config {
        target: Target {
            x_m: 100.0,
            ..Target::default()
        },
        duration_s: 1.0,
        ..Config::default()
    })?;
    assert_eq!(frame.stats.shots, 0);
    Ok(())
}

#[test]
fn oblique_intercepts_track_without_firing() {
    // Fast rotation and a large radius make the cheap slot selection differ
    // from the final incidence. The controller must obey the final gate.
    let mut held = 0;
    for phase in 0..180 {
        let target = Target {
            x_m: 3.0,
            theta_rad: f64::from(phase) * 2.0 * PI / 180.0,
            omega_rad_s: 20.0,
            ..Target::default()
        };
        let mut input = input_at(0.30, 0.25, target);
        input.size.radius_m = 1.0;
        if let Some(Command {
            yaw_rad,
            pitch_rad,
            fire: false,
        }) = aim(&input, &mut Gizmos::default())
        {
            assert!(yaw_rad.is_finite() && pitch_rad.is_finite());
            held += 1;
        }
    }
    assert!(held > 0, "expected converged oblique cases to hold fire");
}

#[test]
fn debug_impact_matches_predicted_center_and_uses_absolute_time() -> Result<(), Box<dyn Error>> {
    let target = Target {
        vx_m_s: 0.8,
        vy_m_s: -0.5,
        omega_rad_s: 7.0,
        ..Target::default()
    };
    let input = input_at(1.0, 0.75, target);
    let mut debug = Gizmos::default();
    aim(&input, &mut debug).ok_or("reachable moving target")?;
    let predicted = debug
        .points
        .iter()
        .find(|p| p.label == "Predicted center")
        .ok_or("predicted marker")?;
    let impact = debug
        .points
        .iter()
        .find(|p| p.label == "Impact")
        .ok_or("impact marker")?;
    let residual = predicted
        .position_m
        .into_iter()
        .zip(impact.position_m)
        .map(|(a, b)| (a - b).powi(2))
        .sum::<f64>()
        .sqrt();
    assert!(residual < 1e-4, "inconsistent fixed point: {residual}m");
    let trajectory = aiming_sim::Trajectory::solve(predicted.position_m).ok_or("trajectory")?;
    let impact_text = &debug
        .overlays
        .iter()
        .find(|(name, _)| name == "Impact time")
        .ok_or("time overlay")?
        .1;
    let impact_time: f64 = impact_text.trim_end_matches(" s").parse()?;
    assert!(
        (impact_time - input.now_s - physics::LAUNCH_DELAY_S - trajectory.flight_time_s).abs()
            < 0.00006
    );
    Ok(())
}
