use rand::{thread_rng, Rng};

use crate::{Isometry2, Point2};
use std::time::Duration;

pub(super) const LOCATE_TOPIC: &str = "locate";

pub(super) struct Locator {
    pose: Isometry2<f32>,
    period: Duration,
    deadline: Duration,
}

impl Locator {
    #[inline]
    pub fn new(pose: Isometry2<f32>, frequency: f32) -> Self {
        Self {
            pose,
            period: Duration::from_secs_f32(1.0 / frequency),
            deadline: Duration::ZERO,
        }
    }

    pub fn update(&mut self, time: Duration, pose: Isometry2<f32>) -> Option<Point2<f32>> {
        if time > self.deadline {
            self.deadline = time + self.period;
            let v = (pose * self.pose).translation.vector;
            Some(point!(
                v[0] + thread_rng().gen_range(-0.02..0.02),
                v[1] + thread_rng().gen_range(-0.02..0.02)
            ))
        } else {
            None
        }
    }
}
