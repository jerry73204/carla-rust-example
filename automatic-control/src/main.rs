use anyhow::{Context, Result};
use carla::{
    client::{ActorBase, Client, Vehicle},
    rpc::{EpisodeSettings, VehicleAckermannControl},
};
use clap::Parser;
use nalgebra::{UnitQuaternion, Vector3};
use rand::prelude::*;
use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    time::Duration,
};

fn main() -> Result<()> {
    let opts = Opts::parse();

    // Create a random generator
    let mut rng = rand::thread_rng();

    // Connect to the client and retrieve the world object
    let client = Client::connect(&opts.addr, opts.port, None);

    // Set the world
    let mut world = match &opts.world {
        Some(world) => client.load_world(world),
        None => client.world(),
    };

    // Set a/synchronous mode
    world.apply_settings(
        &EpisodeSettings {
            synchronous_mode: true, // Enables synchronous mode
            fixed_delta_seconds: Some(0.05),
            ..world.settings()
        },
        Duration::ZERO,
    );

    // Choose a start point randomly
    let map = world.map();
    let spawn_points: Vec<_> = map.recommended_spawn_points().iter().collect();
    let start_point = spawn_points.choose(&mut rng).unwrap();

    eprintln!("Spawn a vehicle at {start_point}");

    let stop = Arc::new(AtomicBool::new(false));
    {
        let stop = stop.clone();

        ctrlc::set_handler(move || {
            stop.store(true, Ordering::SeqCst);
        })
        .with_context(|| "Error setting Ctrl-C handler")?;
    }

    // Spawn vehicles
    let vblu = world
        .blueprint_library()
        .find("vehicle.tesla.model3")
        .unwrap();
    let vehicle: Vehicle = world.spawn_actor(&vblu, start_point)?.try_into().unwrap();
    vehicle.set_autopilot(true);

    while !stop.load(Ordering::SeqCst) {
        // Get the current waypoint.
        let vehicle_transform = vehicle.transform();
        let vehicle_location = vehicle_transform.translation;

        eprintln!("vehicle drives at {vehicle_location}");

        let Some(curr_waypoint) = map.waypoint(&vehicle_location) else {
            vehicle.set_transform(start_point);
            continue;
        };

        // Choose a next waypoint randomly
        let next_waypoints: Vec<_> = curr_waypoint.next(1.0).iter().collect();
        let Some(next_waypoint) = next_waypoints.choose(&mut rng) else {
            vehicle.set_transform(start_point);
            continue;
        };

        // Compute the offset vector from the car to the next waypoint
        let next_location = next_waypoint.transofrm().translation;
        let dir = next_location.vector - vehicle_transform.translation.vector;

        // Create a rotation that looks at the direction of `dir`.
        let up = Vector3::z();
        let target_rot = UnitQuaternion::look_at_lh(&dir, &up);

        // Compute the angle offset from the vehicle heading to the target direction.
        let heading_offset = vehicle_transform.rotation.angle_to(&target_rot);

        // Compute the steering speed
        let steer_speed = if heading_offset.abs() < 3f32.to_radians() {
            0.0
        } else if heading_offset > 0.0 {
            1f32.to_radians()
        } else {
            -1f32.to_radians()
        };

        // Get the current car speed.
        let vehicle_speed = vehicle.velocity().norm();

        // Compute the acceleration
        let acceleration = if vehicle_speed < 5.0 { 1.0 } else { 0.0 };

        // Apply the control to the car
        let control = VehicleAckermannControl {
            steer: heading_offset,
            steer_speed,
            speed: opts.target_speed,
            acceleration,
            jerk: 0.0,
        };
        vehicle.apply_ackermann_control(&control);

        world.tick();
    }

    // Restore the world settings
    world.apply_settings(
        &EpisodeSettings {
            synchronous_mode: false,
            fixed_delta_seconds: None,
            ..world.settings()
        },
        Duration::ZERO,
    );

    Ok(())
}

#[derive(Parser)]
struct Opts {
    #[clap(long, default_value = "localhost")]
    pub addr: String,

    #[clap(long, default_value = "2000")]
    pub port: u16,

    #[clap(long)]
    pub world: Option<String>,

    #[clap(long, default_value = "20.0")]
    pub target_speed: f32,
}
