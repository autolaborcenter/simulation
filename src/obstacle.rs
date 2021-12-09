use crate::{isometry, vector, Isometry2, Point2, Vector2};
use parry2d::bounding_volume::AABB;

mod outlines;
mod ray_cast;
mod simplify;

use ray_cast::Segment;

pub(super) use outlines::*;
pub(super) use ray_cast::ray_cast;
pub(super) use simplify::*;

/// 障碍物对象
#[derive(Clone)]
pub struct Obstacle {
    aabb: AABB,
    pub vertex: Vec<Point2<f32>>, // 顺时针排列的顶点
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

    /// 判断点是否在障碍物内
    pub fn contains(&self, p: Point2<f32>) -> bool {
        self.aabb.contains_local_point(&p) && {
            let mut p0 = self.vertex[self.vertex.len() - 1];
            self.vertex.iter().all(|p1| {
                let result = !is_left(p0, *p1, p);
                p0 = *p1;
                result
            })
        }
    }

    /// 计算线段与障碍物交点，返回可能绕过障碍物的倒序的一列点
    pub fn go_through(
        &self,
        path: &mut impl Iterator<Item = (usize, Point2<f32>)>,
        trend: &mut f32,
    ) -> Point2<f32> {
        // 找到离开位置，如果可直达，直接去
        while let Some((_, p)) = path.next() {
            if !self.contains(p) {
                if Segment(point!(0, 0), p)
                    .intersection_with_polygon(&self.vertex)
                    .is_none()
                {
                    return p;
                } else {
                    break;
                }
            }
        }
        // 计算方向上下界
        let front = self.vertex[0];
        let front = (front[1].atan2(front[0]), front);
        let (left, right) =
            self.vertex
                .iter()
                .skip(1)
                .fold((front, front), |(mut min, mut max), p| {
                    let angle = p[1].atan2(p[0]);
                    if angle < min.0 {
                        min = (angle, *p);
                    } else if angle > max.0 {
                        max = (angle, *p);
                    }
                    (min, max)
                });
        // 选需要转向更小的一边
        if left.0.abs() * *trend < right.0.abs() {
            *trend *= 0.998;
            left.1
        } else {
            *trend *= 1.002;
            right.1
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
        point!(cos * self.rho, sin * self.rho)
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
    /// 法向量
    #[inline]
    fn normal(v: Vector2<f32>) -> Vector2<f32> {
        vector(-v[1], v[0])
    }

    // 占个位置
    let mut result = vec![v[0] + len * normal((v[1] - v[0]).normalize())];
    // 扩张顶点
    result.extend(v.windows(3).flat_map(|triple| {
        let c = triple[1];
        let d0 = (triple[0] - c).normalize();
        let d1 = (triple[2] - c).normalize();
        let cross = cross_numeric(d0, d1);
        // 锐角切成直角
        if cross <= 0.0 && d0.dot(&d1).is_sign_positive() {
            vec![
                c - len * normal(d0),
                c - len * (d0 + d1).normalize(),
                c + len * normal(d1),
            ]
        } else {
            vec![c + (d0 + d1) * len / cross]
        }
    }));
    result.push(v[v.len() - 1] - len * normal((v[v.len() - 2] - v[v.len() - 1]).normalize()));
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
    const A: Point2<f32> = point!(0, 0);
    const B: Point2<f32> = point!(1, 0);
    const C: Point2<f32> = point!(2, 1);
    const D: Point2<f32> = point!(2, -1);
    assert!(is_left(A, B, C));
    assert!(!is_left(A, B, D));
}
