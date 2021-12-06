use crate::{isometry, point, vector, Isometry2, Point2, Vector2};
use parry2d::bounding_volume::AABB;

mod outlines;
mod ray_cast;
mod simplify;

pub(super) use outlines::*;
pub(super) use ray_cast::ray_cast;
use ray_cast::Segment;
pub(super) use simplify::*;

/// 障碍物对象
#[derive(Clone)]
pub struct Obstacle {
    aabb: AABB,
    pub vertex: Vec<Point2<f32>>,
}

impl Obstacle {
    /// 构造障碍物对象
    pub fn new(sensor_on_robot: Isometry2<f32>, wall: &[Point2<f32>], width: f32) -> Option<Self> {
        if wall.is_empty() {
            None
        } else {
            let vertex = enlarge(wall, width * 0.5)
                .into_iter()
                .map(|p| sensor_on_robot * p)
                .collect::<Vec<_>>();
            Some(Self {
                aabb: AABB::from_points(vertex.iter()),
                vertex,
            })
        }
    }

    /// 计算线段与障碍物交点
    pub fn intersection(&self, p0: Point2<f32>, p1: Point2<f32>) -> Option<(usize, f32)> {
        // p1 在内部
        if self.aabb.contains_local_point(&p1) {
            Segment(p0, p1).intersection_with_polygon(&self.vertex)
        } else {
            None
        }
    }
}

#[derive(Clone, Copy)]
pub(super) struct Polar {
    rho: f32,
    theta: f32,
}

impl From<Point2<f32>> for Polar {
    #[inline]
    fn from(p: Point2<f32>) -> Self {
        Self {
            rho: p.coords.norm(),
            theta: p.coords[1].atan2(p.coords[0]),
        }
    }
}

impl Polar {
    #[inline]
    pub fn to_point(&self) -> Point2<f32> {
        let (sin, cos) = self.theta.sin_cos();
        point(cos * self.rho, sin * self.rho)
    }

    #[inline]
    pub fn reset_radius_of_point(p: Point2<f32>, r: f32) -> Point2<f32> {
        Polar {
            rho: r,
            theta: p[1].atan2(p[0]),
        }
        .to_point()
    }
}

/// 扩张凸折线
fn enlarge(v: &[Point2<f32>], len: f32) -> Vec<Point2<f32>> {
    let radius = v[0].coords.norm();
    // 占个位置
    let mut result = vec![point(0.0, 0.0)];
    // 扩张顶点
    result.extend(v.windows(3).flat_map(|triple| {
        let c = triple[1];
        let d0 = (triple[0] - c).normalize();
        let d1 = (triple[2] - c).normalize();
        let cross = cross_numeric(d0, d1);
        // 锐角切成直角
        if cross.is_sign_negative() && d0.dot(&d1).is_sign_positive() {
            /// 法向量
            #[inline]
            fn normal(v: Vector2<f32>) -> Vector2<f32> {
                vector(-v[1], v[0])
            }

            vec![
                c - len * normal(d0),
                c - len * (d0 + d1).normalize(),
                c + len * normal(d1),
            ]
        } else {
            vec![c + (d0 + d1) * len / cross]
        }
    }));
    // 投影第一个点
    let p = result[1];
    result[0] = Polar::reset_radius_of_point(p, radius);
    // 投影最后一个点
    let p = *result.last().unwrap();
    result.push(Polar::reset_radius_of_point(p, radius));

    result
}

/// 叉积 === 两向量围成平行四边形面积
#[inline]
fn cross_numeric(v0: Vector2<f32>, v1: Vector2<f32>) -> f32 {
    v0[1] * v1[0] - v0[0] * v1[1]
}

/// `c` 在 `ab` 左侧
#[inline]
fn is_left(a: Point2<f32>, b: Point2<f32>, c: Point2<f32>) -> bool {
    cross_numeric(b - a, c - b).is_sign_negative()
}

#[test]
fn test_is_left() {
    const A: Point2<f32> = point(2.3799877, -1.7332762);
    const B: Point2<f32> = point(1.0, 0.0);
    const C: Point2<f32> = point(2.0, 1.0);
    const D: Point2<f32> = point(2.0, -1.0);
    assert!(is_left(A, B, C));
    assert!(!is_left(A, B, D));
}
