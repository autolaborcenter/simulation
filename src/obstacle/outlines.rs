use std::f32::consts::FRAC_PI_8;

use crate::{isometry, polar::Polar, Isometry2, Point2};

pub(crate) const OBSTACLES_TOPIC: &str = "obstacles";
pub(crate) const LIDAR_TOPIC: &str = "lidar";
pub(crate) const MOVING_TOPIC: &str = "moving";

#[allow(dead_code)]
pub(crate) const 三轮车: [Point2<f32>; 5] = [
    point!(1.0, 0.0),
    point!(0.0, 0.75),
    point!(-2.0, 0.75),
    point!(-2.0, -0.75),
    point!(0.0, -0.75),
];

#[allow(dead_code)]
pub(crate) const 崎岖轮廓: [Point2<f32>; 12] = [
    point!(0.0, 0.0),
    point!(2.0, -3.0),
    point!(4.0, -2.0),
    point!(2.0, -2.0),
    point!(3.0, -1.2),
    point!(2.0, -0.4),
    point!(4.0, 0.0),
    point!(2.0, 0.4),
    point!(3.0, 1.2),
    point!(2.0, 2.0),
    point!(4.0, 2.0),
    point!(2.0, 3.0),
];

#[allow(dead_code)]
pub(crate) const 墙: [Point2<f32>; 4] = [
    point!(3.0, 0.1),
    point!(3.0, -0.1),
    point!(-3.0, -0.1),
    point!(-3.0, 0.1),
];

pub(crate) struct Person {
    frames: u8,
    len: f32,

    pose: Isometry2<f32>,
    pub next: f32,
    state: State,
    step: u8,
}

enum State {
    Left,
    Right,
}

impl Person {
    #[inline]
    pub const fn new(frames: u8, len: f32, pose: Isometry2<f32>) -> Self {
        Self {
            frames,
            len,
            pose,
            next: 0.0,
            state: State::Left,
            step: 0,
        }
    }

    pub fn update(&mut self) {
        self.step += 1;
        if self.step > self.frames {
            self.step = 0;
            self.state = match self.state {
                State::Left => State::Right,
                State::Right => State::Left,
            };
            let (sin, cos) = self.next.sin_cos();
            self.next = 0.0;
            self.pose *= isometry(self.len, 0.0, cos, sin);
        }
    }

    pub fn to_points(&self) -> (Vec<Point2<f32>>, Vec<Point2<f32>>) {
        let one = (0..16)
            .map(|i| {
                Polar {
                    rho: 0.075,
                    theta: i as f32 * FRAC_PI_8,
                }
                .into()
            })
            .collect::<Vec<Point2<f32>>>();
        let x = self.len * (((self.step * 2) as f32 / self.frames as f32) - 1.0);
        let (left, right) = match self.state {
            State::Left => (
                self.pose * isometry(x, 0.1, 1.0, 0.0),
                self.pose * isometry(0.0, -0.1, 1.0, 0.0),
            ),
            State::Right => (
                self.pose * isometry(0.0, 0.1, 1.0, 0.0),
                self.pose * isometry(x, -0.1, 1.0, 0.0),
            ),
        };
        (
            one.iter().map(|p| left * p).collect(),
            one.iter().map(|p| right * p).collect(),
        )
    }
}
