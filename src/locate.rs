use crate::{gaussian::Gaussian, Isometry2, Point2};
use rand::{thread_rng, RngCore};
use std::{collections::VecDeque, time::Duration};

pub(crate) const LOCATION_TOPIC: &str = "location";
pub(crate) const ODOMETRY_TOPIC: &str = "odometry";

pub(crate) struct Location {
    beacon_on_robot: Isometry2<f32>,
    gaussian: Gaussian,
    period: Duration,
    delay: Duration,
    deadline: Duration,
    queue: VecDeque<(Duration, Isometry2<f32>)>,
}

impl Location {
    #[inline]
    pub fn new(pose: Isometry2<f32>, delay: Duration, frequency: f32, sigma: f32) -> Self {
        Self {
            beacon_on_robot: pose,
            gaussian: Gaussian::new(0.0, sigma),
            period: Duration::from_secs_f32(1.0 / frequency),
            delay,
            deadline: Duration::ZERO,
            queue: Default::default(),
        }
    }

    pub fn update(
        &mut self,
        now: Duration,
        pose: Isometry2<f32>,
    ) -> Option<(Duration, Point2<f32>)> {
        self.queue.push_front((now + self.delay, pose));
        if now >= self.deadline {
            let mut result = None;
            while self.queue.back().filter(|(t, _)| *t <= now).is_some() {
                let len = self.queue.len() as u32;
                let (time, pose) = self.queue.pop_back().unwrap();
                // 越新的越可能输出
                if thread_rng().next_u32() % len <= 1 {
                    let v = (pose * self.beacon_on_robot).translation.vector;
                    result = Some((
                        time,
                        point!(v[0] + self.gaussian.next(), v[1] + self.gaussian.next()),
                    ));
                }
            }
            if result.is_some() {
                self.deadline = now + self.period;
            }
            result
        } else {
            None
        }
    }
}
