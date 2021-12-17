use crate::{gaussian::Gaussian, Isometry2, Point2};
use std::time::Duration;

pub(super) const LOCATE_TOPIC: &str = "locate";

pub(super) struct Locator {
    beacon_on_robot: Isometry2<f32>,
    gaussian: Gaussian,
    period: Duration,
    deadline: Duration,
}

impl Locator {
    #[inline]
    pub fn new(pose: Isometry2<f32>, frequency: f32, sigma: f32) -> Self {
        Self {
            beacon_on_robot: pose,
            gaussian: Gaussian::new(0.0, sigma),
            period: Duration::from_secs_f32(1.0 / frequency),
            deadline: Duration::ZERO,
        }
    }

    pub fn update(&mut self, time: Duration, pose: Isometry2<f32>) -> Option<Point2<f32>> {
        if time > self.deadline {
            self.deadline = time + self.period;
            let v = (pose * self.beacon_on_robot).translation.vector;
            Some(point!(
                v[0] + self.gaussian.next(),
                v[1] + self.gaussian.next()
            ))
        } else {
            None
        }
    }
}
