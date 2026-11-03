//! Summarize complete runs using the public, unchanged evaluator (including flight drain).
use aiming_sim::{
    aim, physics,
    sim::{Config, Simulation},
};
use anyhow::{Context, Result, ensure};
use serde_json::json;
use std::{env, fs, path::PathBuf};

fn main() -> Result<()> {
    let mut paths: Vec<PathBuf> = env::args_os().skip(1).map(PathBuf::from).collect();
    if paths.is_empty() {
        paths = ["stationary", "spinning", "moving"]
            .map(|name| PathBuf::from(format!("scenarios/{name}.json")))
            .to_vec();
    }
    let mut reports = Vec::new();
    for path in paths {
        let config: Config = serde_json::from_reader(
            fs::File::open(&path).with_context(|| format!("read {}", path.display()))?,
        )?;
        let mut simulation = Simulation::new(config)?;
        let end_s = config.duration_s + physics::LAUNCH_DELAY_S + physics::MAX_FLIGHT_S;
        let mut ticks = 0_u64;
        let mut aim_ticks = 0_u64;
        let mut fire_requests = 0_u64;
        let mut max_endpoint_residual_m = 0.0_f64;
        let final_frame = loop {
            let frame = simulation.step(&mut |input, debug| {
                if input.now_s < config.duration_s {
                    aim(input, debug)
                } else {
                    None
                }
            });
            ticks += 1;
            if let Some(command) = frame.command {
                aim_ticks += 1;
                fire_requests += u64::from(command.fire);
                let predicted = frame
                    .gizmos
                    .points
                    .iter()
                    .find(|point| point.label == "Predicted center");
                let impact = frame
                    .gizmos
                    .points
                    .iter()
                    .find(|point| point.label == "Impact");
                if let (Some(predicted), Some(impact)) = (predicted, impact) {
                    let residual = predicted
                        .position_m
                        .into_iter()
                        .zip(impact.position_m)
                        .map(|(a, b)| (a - b).powi(2))
                        .sum::<f64>()
                        .sqrt();
                    max_endpoint_residual_m = max_endpoint_residual_m.max(residual);
                }
            }
            if frame.time_s >= end_s {
                break frame;
            }
        };
        let stats = final_frame.stats;
        let resolved = stats.hits + stats.misses;
        ensure!(
            resolved == stats.shots,
            "unsettled projectiles in {}",
            path.display()
        );
        reports.push(json!({
            "scenario": path, "config": config, "final_time_s": final_frame.time_s,
            "ticks": ticks, "aim_ticks": aim_ticks, "fire_requests": fire_requests,
            "stats": stats, "settled_shots": resolved,
            "hit_rate": (resolved > 0).then(|| stats.hits as f64 / resolved as f64),
            "max_endpoint_residual_m": max_endpoint_residual_m,
        }));
    }
    println!("{}", serde_json::to_string_pretty(&reports)?);
    Ok(())
}
