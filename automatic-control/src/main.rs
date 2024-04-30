use anyhow::{Context, Result};
use carla::{
    client::{ActorBase, Client, Vehicle},
    rpc::{EpisodeSettings, VehicleAckermannControl},
};
use clap::Parser;
use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use noisy_float::prelude::*;
use std::{
    f32::consts::PI,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    time::Duration,
};

fn main() -> Result<()> {
    let opts = Opts::parse();

    // Connect to the client and retrieve the world object
    let client = Client::connect(&opts.addr, opts.port, None);

    // Set the world
    let mut world = match &opts.world {
        Some(world) => client.load_world(world),
        None => {
            // It causes the client to crash.
            // client.reload_world()

            client.world()
        }
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

    // spawn the vehicle in ...
    let start_point = Isometry3 {
        translation: Translation3::new(83.075226, 13.414804, 0.600000),
        rotation: UnitQuaternion::from_euler_angles(0.0, 0.0, -179.840_79_f32.to_radians()),
    };
    eprintln!("Spawn a vehicle at {start_point}");

    // Register a Ctrl-C handler
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
    let vehicle: Vehicle = world.spawn_actor(&vblu, &start_point)?.try_into().unwrap();
    vehicle.set_autopilot(false);

    let spectator = world.spectator();

    while !stop.load(Ordering::SeqCst) {
        // Get the current waypoint.
        let vehicle_transform = vehicle.transform();
        let vehicle_location = vehicle_transform.translation;

        let Some(curr_waypoint) = map.waypoint(&vehicle_location) else {
            vehicle.set_transform(&start_point);
            continue;
        };

        // Choose a next waypoint
        let Some(next_waypoint) = curr_waypoint.next(1.0).get(0) else {
            vehicle.set_transform(&start_point);
            continue;
        };

        // Set the spectator viewpoint
        let s_point = vehicle_transform * Translation3::new(-10.0, 0.0, 7.0);
        spectator.set_transform(&s_point);

        // Compute the displacement vector from the car to the next
        // waypoint.
        let next_location = next_waypoint.transform().translation;
        let dir = next_location.vector - vehicle_transform.translation.vector;

        // Compute the heading offset towards the next waypoint.
        let heading_offset = {
            let (_, _, current_yaw) = vehicle_transform.rotation.euler_angles();
            let target_yaw = (dir.y).atan2(dir.x);

            let current_yaw = current_yaw; // Change range [0, 2π] to [-π, π]
            let offset = target_yaw - current_yaw;

            if offset >= PI {
                offset - PI * 2.0
            } else if offset <= -PI {
                offset + PI * 2.0
            } else {
                offset
            }
        }
        .to_degrees();

        // Compute the steering ratio
        let steer = {
            let physics_control = vehicle.physics_control();
            let max_steer_angle = physics_control
                .wheels
                .iter()
                .map(|wheel| r32(wheel.max_steer_angle))
                .max()
                .expect("Unable to obtain max steering angle from the vehicle")
                .raw();
            (heading_offset / max_steer_angle).clamp(-1.0, 1.0)
        };

        // Compute the steering speed
        let steer_speed = if heading_offset.abs() < 3.0 {
            0.0
        } else if heading_offset > 0.0 {
            0.1
        } else {
            -0.1
        };

        // Get the current car speed.
        let vehicle_speed = vehicle.velocity().norm();

        // Compute the acceleration
        let acceleration = if vehicle_speed < 5.0 { 1.0 } else { 0.0 };

        // Apply the control to the car
        let control = VehicleAckermannControl {
            //TODO: the parameter of 'steer' has bug
            steer,
            steer_speed,
            speed: opts.target_speed * 10.0 / 36.0,
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

    #[clap(long, default_value = "5.0")]
    pub target_speed: f32,
}
