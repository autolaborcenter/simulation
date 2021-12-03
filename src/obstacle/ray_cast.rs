use super::{cross_numeric, Polar};
use crate::{point, vector, Isometry2, Point2, Sector};
use parry2d::bounding_volume::AABB;
use rand::{thread_rng, Rng};
use std::f32::consts::PI;

type Point = Point2<f32>;

/// 放出射线生成点云
pub(crate) fn ray_cast(
    pose: Isometry2<f32>,
    obstacles: &Vec<Vec<Point2<f32>>>,
    range: Sector,
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
            .map(|v| v.iter().map(|p| to_local * p).collect::<Vec<_>>())
            .collect::<Vec<_>>()
    };
    //
    let mut result = Vec::new();
    const ORIGIN: Point2<f32> = point(0.0, 0.0);
    const STEP: f32 = PI / 360.0;
    let dir_range = range.angle * 0.5;
    let mut theta = -dir_range;
    while theta <= dir_range {
        let (sin, cos) = theta.sin_cos();
        let ray = Segment(
            ORIGIN,
            Point {
                coords: vector(cos, sin) * range.radius,
            },
        );
        let rho = obstacles
            .iter()
            .filter_map(|c| ray.ray_cast(c))
            .fold(f32::INFINITY, |min, l| f32::min(min, l))
            + thread_rng().gen_range(-0.01..0.01);
        if rho.is_normal() {
            result.push(Polar { rho, theta });
        }
        theta += STEP;
    }
    result
}

/// 线段
pub(super) struct Segment(pub Point, pub Point);

impl Segment {
    pub fn len(&self) -> f32 {
        (self.0 - self.1).norm()
    }

    /// 两线段是否相交
    pub fn intersection(&self, others: &Segment) -> Option<f32> {
        let Self(a, b) = self;
        let Self(c, d) = others;
        let ac = c - a;
        let bc = c - b;
        let ad = d - a;
        let bd = d - b;

        let abc = cross_numeric(ac, bc);
        let abd = cross_numeric(ad, bd);
        if abc * abd >= -f32::EPSILON {
            return None;
        }

        let acd = cross_numeric(ac, ad);
        let bcd = abc - abd + acd;
        if acd * bcd >= -f32::EPSILON {
            return None;
        }

        Some(acd / (abd - abc))
    }

    /// 如果射线穿过多边形，计算首次穿过的长度
    pub fn ray_cast(&self, polygon: &[Point]) -> Option<f32> {
        match polygon {
            &[] | &[_] => None,
            &[c, d] => self.intersection(&Self(c, d)),
            _ => {
                let mut p0 = polygon[polygon.len() - 1];
                polygon
                    .iter()
                    .filter_map(|p| {
                        let result = self.intersection(&Self(p0, *p));
                        p0 = *p;
                        result
                    })
                    .reduce(|a, b| f32::min(a, b))
            }
        }
        .map(|t| self.len() * t)
    }
}

#[test]
fn test_intersection() {
    use crate::point;
    const S0: Segment = Segment(point(0.0, 0.0), point(1.0, 0.0));
    const S1: Segment = Segment(point(0.0, -1.0), point(1.0, 1.0));
    assert_eq!(Some(0.5), S0.intersection(&S1));
}
