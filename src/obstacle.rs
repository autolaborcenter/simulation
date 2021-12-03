use crate::{isometry, point, vector, Point2, Vector2};
use std::{collections::VecDeque, f32::consts::PI};

mod outlines;
mod ray_cast;

pub(super) use outlines::*;
pub(super) use ray_cast::ray_cast;

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
    pub fn to_point(&self) -> Point2<f32> {
        let (sin, cos) = self.theta.sin_cos();
        point(cos * self.rho, sin * self.rho)
    }
}

/// 线段拟合
pub(super) fn fit(
    points: impl IntoIterator<Item = Point2<f32>>,
    radius: f32,
    max_len: f32,
    max_diff: f32,
) -> Vec<Vec<Point2<f32>>> {
    let mut source = points.into_iter();
    let mut result = vec![];
    if let Some(point) = source.next() {
        let polar = Polar::from(point);
        // 最后一点的极坐标，用于分集
        let mut last = polar;
        // 最后一个线段上所有点
        let mut current = vec![point];
        // 初始化折线
        result.push(vec![
            Polar {
                rho: radius,
                theta: polar.theta,
            }
            .to_point(),
            point,
        ]);

        for point in source {
            let polar = Polar::from(point);
            // 最后一条折线
            let tail = result.last_mut().unwrap();
            // 判断与上一个点形成的狭缝能否通过
            let len = {
                let rho = f32::max(polar.rho, last.rho);
                let theta = (polar.theta - last.theta).abs();
                2.0 * rho * (theta * 0.5).sin()
            };
            // 可以通过，分割
            if len > max_len {
                // 保存上一个线段尾
                if current.len() > 1 {
                    let last = *current.last().unwrap();
                    tail.push(last);
                    tail.push(Point2 {
                        coords: last.coords.normalize() * radius,
                    });
                }
                current.clear();
                current.push(point);
                // 初始化折线
                result.push(vec![
                    Polar {
                        rho: radius,
                        theta: polar.theta,
                    }
                    .to_point(),
                    point,
                ]);
            }
            // 不可通过
            else {
                // 构造线段坐标系
                let seg = (*current.first().unwrap() - point).normalize();
                let tr = isometry(point[0], point[1], seg[0], seg[1]).inverse();
                let mut min = 0.0;
                let mut max = 0.0;
                // 超界，折断线段
                let mut outline = false;
                // 折返，不改变线段端点
                let mut backfolding = false;
                // 验证线段上所有点仍在线段上
                for p in current[1..].iter().rev() {
                    let p = tr * p;
                    backfolding |= p[0] < 0.0;
                    let y = p[1];
                    let updated = if y < min {
                        min = y;
                        true
                    } else if y > max {
                        max = y;
                        true
                    } else {
                        false
                    };
                    if updated && max - min > max_diff {
                        outline = true;
                        break;
                    }
                }
                // 未出界，更新线段缓存
                if !outline {
                    if backfolding {
                        current.insert(current.len() - 1, point);
                    } else {
                        current.push(point);
                    }
                }
                // 出界，更新折线
                else {
                    let last = *current.last().unwrap();
                    current.clear();
                    current.push(last);
                    current.push(point);
                    tail.push(last);
                }
            }
            last = polar;
        }
        if current.len() > 1 {
            let tail = result.last_mut().unwrap();
            let last = *current.last().unwrap();
            tail.push(last);
            tail.push(Point2 {
                coords: last.coords.normalize() * radius,
            });
        }
    }
    result
}

// 快速凸包
pub(super) fn melkman(src: Vec<Point2<f32>>) -> Vec<Point2<f32>> {
    if src.len() <= 3 {
        src
    } else {
        let mut buf = VecDeque::from_iter(src.iter().take(3).cloned());
        for p in src.into_iter().skip(3) {
            // 复制最后一点形成闭环
            let last = *buf.back().unwrap();
            buf.push_front(last);
            // 新点在队头线段的左侧
            while !is_left(buf[1], buf[0], p) {
                buf.pop_front();
            }
            // 新点在队尾线段的右侧
            while is_left(buf[buf.len() - 2], buf[buf.len() - 1], p) {
                buf.pop_back();
            }
            // 添加新点
            buf.push_back(p);
        }
        buf.into_iter().collect()
    }
}

/// 扩张凸多边形
///
/// 视作折线且两个端点将被丢弃
fn enlarge(v: &[Point2<f32>], len: f32) -> Vec<Point2<f32>> {
    v.windows(3)
        .flat_map(|triple| {
            let c = triple[1];
            let d0 = (triple[0] - c).normalize();
            let d1 = (triple[2] - c).normalize();
            let cross = cross_numeric(d0, d1);
            // 锐角切成直角
            if cross.is_sign_negative() && d0.dot(&d1).is_sign_positive() {
                vec![
                    c - len * normal(d0),
                    c - len * (d0 + d1).normalize(),
                    c + len * normal(d1),
                ]
            } else {
                vec![c + (d0 + d1) * len / cross]
            }
        })
        .collect::<Vec<_>>()
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

/// 法向量
#[inline]
fn normal(v: Vector2<f32>) -> Vector2<f32> {
    vector(-v[1], v[0])
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
