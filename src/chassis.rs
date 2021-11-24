﻿use monitor_tool::Vertex;
use rand::Rng;
use std::f32::consts::FRAC_PI_2;

pub(super) const CHASSIS_TOPIC: &str = "chassis";
pub(super) const ODOMETRY_TOPIC: &str = "odometry";

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
    vertex!(0.25, 0.08; 0, true),
    vertex!(0.12, 0.14; 0, true),
    vertex!(0.10, 0.18; 0, true),
    vertex!(0.10, 0.26; 0, true),
    //
    vertex!(-0.10, 0.26; 0, true),
    vertex!(-0.10, 0.18; 0, true),
    vertex!(-0.25, 0.18; 0, true),
    vertex!(-0.47, 0.12; 0, true),
    //
    vertex!(-0.47, -0.12; 0, true),
    vertex!(-0.25, -0.18; 0, true),
    vertex!(-0.10, -0.18; 0, true),
    vertex!(-0.10, -0.26; 0, true),
    //
    vertex!(0.10, -0.26; 0, true),
    vertex!(0.10, -0.18; 0, true),
    vertex!(0.12, -0.14; 0, true),
    vertex!(0.25, -0.08; 0, true),
    //
    vertex!(0.25, 0.08; 0, false),
];

pub(super) const RUDDER: [Vertex; 5] = [
    vertex!(-0.075, 0.06; 1, true),
    vertex!(-0.075, -0.06; 1, true),
    vertex!(0.075, -0.06; 1, true),
    vertex!(0.075, 0.06; 1, true),
    //
    vertex!(-0.075, 0.06; 1, false),
];