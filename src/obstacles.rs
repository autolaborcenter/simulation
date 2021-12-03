use super::{point, vector};
use parry2d::{
    bounding_volume::AABB,
    na::{Point2, Vector2},
};
use pm1_control_model::{isometry, Isometry2};
use rand::{thread_rng, Rng};
use std::{collections::VecDeque, f32::consts::PI};

mod segment;

use segment::Segment;

pub(super) const OBSTACLES_TOPIC: &str = "obstacles";
pub(super) const LIDAR_TOPIC: &str = "lidar";

#[allow(dead_code)]
pub(super) const TRICYCLE_OUTLINE: [Point2<f32>; 5] = [
    point(1.0, 0.0),
    point(0.0, 0.75),
    point(-2.0, 0.75),
    point(-2.0, -0.75),
    point(0.0, -0.75),
];

#[allow(dead_code)]
pub(super) const 崎岖轮廓: [Point2<f32>; 12] = [
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

/// 生成点云
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
            .map(|v| v.iter().map(|p| to_local * p).collect::<Vec<_>>())
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
        let ray = Segment(ORIGIN, point(cos * radius, sin * radius));
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
                // 初始化折线
                current.clear();
                current.push(point);
                // 只有一点的折线直接丢弃
                if tail.len() < 3 {
                    result.pop();
                }
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
pub(super) fn enlarge(mut v: Vec<Point2<f32>>, len: f32) -> Vec<Point2<f32>> {
    v.extend_from_within(..2);
    v.windows(3)
        .rev()
        .flat_map(|triple| {
            let c = triple[1];
            let d0 = (triple[0] - c).normalize();
            let d1 = (triple[2] - c).normalize();
            let cross = cross_numeric(d0, d1);
            // 锐角切成直角
            if cross.is_sign_negative() && d0.dot(&d1).is_sign_positive() {
                vec![
                    c + len * normal(d1),
                    c - len * (d0 + d1).normalize(),
                    c - len * normal(d0),
                ]
            } else {
                vec![c + (d0 + d1) * len / cross]
            }
        })
        .collect::<Vec<_>>()
}

/// 叉积 = 两向量围成平行四边形面积
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
