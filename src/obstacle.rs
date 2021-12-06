use crate::{isometry, point, vector, Point2, Vector2};
use std::f32::consts::PI;

mod outlines;
mod ray_cast;
mod simplify;

pub(super) use outlines::*;
pub(super) use ray_cast::ray_cast;
pub(super) use simplify::*;

/// 障碍物对象
#[derive(Clone)]
pub struct Obstacle {
    radius: f32,
    angles: Vec<f32>,
    pub(super) wall: Vec<Point2<f32>>,
}

impl Obstacle {
    /// 构造障碍物对象
    pub fn new(wall: &[Point2<f32>], width: f32, radius: f32) -> Option<Self> {
        let wall = enlarge(wall, width * 0.5);
        if wall.is_empty() {
            None
        } else {
            let mut angles = wall.iter().map(|p| p[1].atan2(p[0])).collect::<Vec<_>>();
            for i in 1..angles.len() - 1 {
                if angles[i] <= angles[i - 1] {
                    angles[i] += PI * 2.0;
                }
            }
            Some(Self {
                radius,
                angles,
                wall,
            })
        }
    }

    /// 判断点是否在障碍物中
    pub fn contains(&self, p: Point2<f32>) -> bool {
        let mut angle = p[1].atan2(p[0]);
        if angle < self.angles[0] {
            angle += PI * 2.0;
        }
        match self.angles.binary_search_by(|r| {
            use std::cmp::Ordering::*;
            if *r < angle {
                Less
            } else if *r > angle {
                Greater
            } else {
                Equal
            }
        }) {
            Ok(i) => p.coords.norm_squared() > self.wall[i].coords.norm_squared(),
            Err(i) => i != 0 && i != self.wall.len() && !is_left(self.wall[i - 1], self.wall[i], p),
        }
    }

    /// 找一个较近的边
    pub fn closer_edge(&self) -> f32 {
        let a = self.angles[0];
        let mut b = *self.angles.last().unwrap();
        if b > PI {
            b -= PI * 2.0;
        }
        if a.abs() < b.abs() {
            a
        } else {
            b
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
