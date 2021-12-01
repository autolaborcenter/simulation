use super::{point, vector};
use parry2d::{
    bounding_volume::AABB,
    na::Point2,
    query::{Ray, RayCast},
    shape::ConvexPolygon,
};
use pm1_control_model::Isometry2;
use rand::{thread_rng, Rng};
use std::f32::consts::PI;

pub(super) const OBSTACLES_TOPIC: &str = "obstacles";
pub(super) const LIDAR_TOPIC: &str = "lidar";

pub(super) const TRICYCLE_OUTLINE: [Point2<f32>; 5] = [
    point(1.0, 0.0),
    point(0.0, 0.75),
    point(-2.0, 0.75),
    point(-2.0, -0.75),
    point(0.0, -0.75),
];

pub(super) struct Polar {
    rho: f32,
    theta: f32,
}

impl Polar {
    pub fn to_point(&self) -> Point2<f32> {
        let (sin, cos) = self.theta.sin_cos();
        point(cos * self.rho, sin * self.rho)
    }
}

pub(super) fn ray_cast(
    pose: Isometry2<f32>,
    obstacles: &Vec<Vec<Point2<f32>>>,
    radius: f32,
    degrees: f32,
) -> Vec<Polar> {
    // 转本地多边形障碍物
    let obstacles = {
        let x0 = pose.translation.vector[0];
        let y0 = pose.translation.vector[1];
        let aabb = AABB::new(point(x0 - 10.0, y0 - 10.0), point(x0 + 10.0, y0 + 10.0));
        let to_local = pose.inverse();
        obstacles
            .iter()
            .filter(|v| v.iter().any(|p| aabb.contains_local_point(p)))
            .filter_map(|v| {
                ConvexPolygon::from_convex_polyline(
                    v.iter().map(|p| to_local * p).collect::<Vec<_>>(),
                )
            })
            .collect::<Vec<_>>()
    };
    //
    let mut result = Vec::new();
    const ORIGIN: Point2<f32> = point(0.0, 0.0);
    const STEP: f32 = PI / 360.0;
    let dir_range = (degrees * 0.5).to_radians();
    let mut theta = -dir_range;
    while theta <= dir_range {
        let (sin, cos) = theta.sin_cos();
        let ray = Ray {
            origin: ORIGIN,
            dir: vector(cos, sin),
        };
        let rho = obstacles
            .iter()
            .filter_map(|c| c.cast_local_ray(&ray, radius, true))
            .fold(f32::INFINITY, |min, l| f32::min(min, l))
            + thread_rng().gen_range(-0.01..0.01);
        if rho.is_normal() {
            result.push(Polar { rho, theta });
        }
        theta += STEP;
    }
    result
}
