use crate::{point, Point2};

pub(crate) const OBSTACLES_TOPIC: &str = "obstacles";
pub(crate) const LIDAR_TOPIC: &str = "lidar";

#[allow(dead_code)]
pub(crate) const TRICYCLE_OUTLINE: [Point2<f32>; 5] = [
    point(1.0, 0.0),
    point(0.0, 0.75),
    point(-2.0, 0.75),
    point(-2.0, -0.75),
    point(0.0, -0.75),
];

#[allow(dead_code)]
pub(crate) const 崎岖轮廓: [Point2<f32>; 12] = [
    point(0.0, 0.0),
    point(2.0, -3.0),
    point(4.0, -2.0),
    point(2.0, -2.0),
    point(3.0, -1.2),
    point(2.0, -0.4),
    point(4.0, 0.0),
    point(2.0, 0.4),
    point(3.0, 1.2),
    point(2.0, 2.0),
    point(4.0, 2.0),
    point(2.0, 3.0),
];
