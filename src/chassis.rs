use crate::{transform, vector, Isometry2};
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

/// 机器人外轮廓
///
/// 基于这个轮廓计算甩尾的最小转弯半径 =>
/// `(r+0.26)^2 + 0.1^2 >= (r+0.12)^2 + 0.47^2` <=> `r>=0.5632`
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

pub(super) fn sector_vertex(
    rgbd_on_chassis: Isometry2<f32>,
    radius: f32,
    degrees: f32,
) -> Vec<Vertex> {
    let (sin, cos) = (degrees.to_radians() * 0.5).sin_cos();
    let d = vector(cos, sin) * radius;
    vec![
        transform(&rgbd_on_chassis, vertex!(3; d[0],  d[1]; 0)),
        transform(
            &rgbd_on_chassis,
            vertex!(3;  0.0,   0.0; Circle, radius; 64),
        ),
        transform(&rgbd_on_chassis, vertex!(3; d[0], -d[1]; 64)),
    ]
}
