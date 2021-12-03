use crate::{isometry, point, vector, Point2, Vector2};
use std::{collections::VecDeque, f32::consts::PI, ops::Range};

mod outlines;
mod ray_cast;

pub(super) use outlines::*;
pub(super) use ray_cast::ray_cast;

/// 障碍物对象
#[derive(Clone)]
pub struct Obstacle {
    pub(super) radius: f32,
    pub(super) angle: Range<f32>,
    pub(super) wall: Vec<Point2<f32>>,
}

impl Obstacle {
    /// 构造障碍物对象
    pub fn new(wall: &[Point2<f32>], width: f32, radius: f32) -> Option<Self> {
        let wall = enlarge(wall, width * 0.5);
        if wall.is_empty() {
            None
        } else {
            let start = wall.first().unwrap();
            let start = start[1].atan2(start[0]);
            let end = wall.last().unwrap().coords;
            let end = end[1].atan2(end[0]);
            Some(Self {
                radius,
                angle: Range {
                    start,
                    end: if end >= start { end } else { end + PI * 2.0 },
                },
                wall,
            })
        }
    }

    pub fn contains(&self, p: Point2<f32>) -> bool {
        let mut angle = p[1].atan2(p[0]);
        if angle < self.angle.start {
            angle += PI * 2.0;
        }
        if angle < self.angle.end {
            false
        } else {
            self.wall
                .windows(2)
                .all(|pair| is_left(pair[0], pair[1], p))
        }
    }
}

#[derive(Clone, Copy)]
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

/// 线段拟合
pub(super) fn fit(
    points: &[Polar],
    radius: f32,
    max_len: f32,
    max_diff: f32,
) -> Vec<Vec<Point2<f32>>> {
    let mut source = points.iter();
    let mut result = vec![];
    if let Some(polar) = source.next() {
        // 最后一点的极坐标，用于分集
        let mut last = *polar;
        let point = polar.to_point();
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

        for polar in source {
            // 最后一条折线
            let tail = result.last_mut().unwrap();

            let point = polar.to_point();
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
            last = *polar;
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
    const A: Point2<f32> = point(0.0, 0.0);
    const B: Point2<f32> = point(1.0, 0.0);
    const C: Point2<f32> = point(2.0, 1.0);
    const D: Point2<f32> = point(2.0, -1.0);
    assert!(is_left(A, B, C));
    assert!(!is_left(A, B, D));
}
