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
        mut p0: Point2<f32>,
        trend: &mut f32,
    ) -> Vec<Point2<f32>> {
        // 计算方向上下界
        let any = self.vertex[0];
        let mut right = (any[1].atan2(any[0]), 0);
        let mut left = right;
        for (i, p) in self.vertex.iter().enumerate().skip(1) {
            let angle = p[1].atan2(p[0]);
            if angle < right.0 {
                right = (angle, i);
            } else if angle > left.0 {
                left = (angle, i);
            }
        }
        // 找到离开位置
        while let Some((_, p1)) = path.next() {
            if let Some((j, k)) = Segment(p0, p1).intersection_with_polygon(&self.vertex) {
                if Segment(point!(0, 0), p1)
                    .intersection_with_polygon(&self.vertex)
                    .is_none()
                {
                    return vec![p1];
                } else {
                    let mut i = if j == 0 { self.vertex.len() - 1 } else { j - 1 };
                    let mut j = j;

                    let seg = self.vertex[j] - self.vertex[i];
                    let l = seg.norm();
                    let ll = self.sum_length(left.1, i) + l * k;
                    let lr = l * (1.0 - k) + self.sum_length(j, right.1);

                    let mut result = vec![self.vertex[i] + seg * k];
                    if ll * *trend < lr {
                        *trend *= 0.999;
                        while i != left.1 {
                            result.push(self.vertex[i]);
                            i = if i == 0 { self.vertex.len() - 1 } else { i - 1 };
                        }
                        result.push(self.vertex[i]);
                    } else {
                        *trend *= 1.001;
                        while j != right.1 {
                            result.push(self.vertex[j]);
                            j = if j == self.vertex.len() - 1 { 0 } else { j + 1 };
                        }
                        result.push(self.vertex[j]);
                    }
                    return result;
                }
            } else {
                p0 = p1;
            }
        }

        vec![
            self.vertex[if left.0.abs() * *trend < right.0.abs() {
                *trend *= 0.999;
                left
            } else {
                *trend *= 1.001;
                right
            }
            .1],
        ]
    }

    fn sum_length(&self, mut begin: usize, end: usize) -> f32 {
        let mut sum = 0.0;
        while begin != end {
            let next = (begin + 1) % self.vertex.len();
            sum += (self.vertex[begin] - self.vertex[next]).norm();
            begin = next;
        }
        sum
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
