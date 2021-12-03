use monitor_tool::{vertex, Vertex};
use rand::Rng;
use std::f32::consts::FRAC_PI_2;

pub(super) const CHASSIS_TOPIC: &str = "chassis";
pub(super) const ODOMETRY_TOPIC: &str = "odometry";

#[allow(dead_code)]
pub(super) fn random_rudder(rudder: &mut f32) {
    let sign = rand::thread_rng().gen_range(-5..=5);
    let delta = if *rudder > 0.0 {
        match sign {
            -5 | -4 => -0.1,
            5 => 0.1,
            _ => 0.0,
        }
    } else {
        match sign {
            -5 => -0.1,
            5 | 4 => 0.1,
            _ => 0.0,
        }
    };
    *rudder = (*rudder + delta).clamp(-FRAC_PI_2, FRAC_PI_2);
}

pub(super) const ROBOT_OUTLINE: [Vertex; 17] = [
    vertex!(0; 0.25, 0.08; 0),
    vertex!(0; 0.12, 0.14; 64),
    vertex!(0; 0.10, 0.18; 64),
    vertex!(0; 0.10, 0.26; 64),
    //
    vertex!(0; -0.10, 0.26; 64),
    vertex!(0; -0.10, 0.18; 64),
    vertex!(0; -0.25, 0.18; 64),
    vertex!(0; -0.47, 0.12; 64),
    //
    vertex!(0; -0.47, -0.12; 64),
    vertex!(0; -0.25, -0.18; 64),
    vertex!(0; -0.10, -0.18; 64),
    vertex!(0; -0.10, -0.26; 64),
    //
    vertex!(0; 0.10, -0.26; 64),
    vertex!(0; 0.10, -0.18; 64),
    vertex!(0; 0.12, -0.14; 64),
    vertex!(0; 0.25, -0.08; 64),
    //
    vertex!(0; 0.25, 0.08; 64),
];

pub(super) const RUDDER: [Vertex; 5] = [
    vertex!(1; -0.075, 0.06; 0),
    vertex!(1; -0.075, -0.06; 64),
    vertex!(1; 0.075, -0.06; 64),
    vertex!(1; 0.075, 0.06; 64),
    //
    vertex!(1; -0.075, 0.06; 64),
];

pub(super) const SIMPLE_OUTLINE: [Vertex; 6] = [
    vertex!(2; 0.32, 0.0; 0),
    vertex!(2; 0.10, 0.26; 64),
    vertex!(2; -0.47, 0.26; 64),
    vertex!(2; -0.47, -0.26; 64),
    vertex!(2; 0.10, -0.26; 64),
    //
    vertex!(2; 0.32, 0.0; 64),
];

pub(super) fn rgbd_bounds(radius: f32, degrees: f32) -> Vec<Vertex> {
    let mut result = Vec::new();
    let (sin, cos) = (degrees.to_radians() * 0.5).sin_cos();
    result.push(vertex!(3; cos * radius, sin * radius; 0));
    result.push(vertex!(3; 0.0, 0.0; Circle, radius; 64));
    result.push(vertex!(3; cos * radius, -sin * radius; 64));
    result
}
