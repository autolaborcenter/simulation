use super::{point, vector};
use parry2d::{
    bounding_volume::AABB,
    na::{Point2, Vector2},
    query::{Ray, RayCast},
    shape::ConvexPolygon,
};
use pm1_control_model::{isometry, Isometry2};
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

/// 线段拟合
pub(super) fn fit(points: &[Polar], max_len: f32, max_diff: f32) -> Vec<Vec<Point2<f32>>> {
    let mut source = points.iter();
    let mut result = vec![];
    if let Some(polar) = source.next() {
        // 最后一点的极坐标，用于分集
        let mut last = *polar;
        // 最后一个线段上所有点
        let mut current = vec![polar.to_point()];
        // 初始化折线
        result.push(current.clone());

        for polar in source {
            // 最后一条折线
            let tail = result.last_mut().unwrap();

            let point = polar.to_point();
            // 判断与上一个点形成的狭缝能否通过
            let len = {
                let rho = f32::max(polar.rho, last.rho);
                let theta = (polar.theta - last.theta).abs();
                rho * (theta * 0.5).sin()
            };
            // 可以通过，分割
            if len > max_len {
                // 保存上一个线段尾
                if current.len() > 1 {
                    tail.push(*current.last().unwrap());
                }
                // 初始化折线
                current.clear();
                current.push(point);
                // 只有一点的折线直接丢弃
                if tail.len() < 2 {
                    result.pop();
                }
                result.push(current.clone());
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
            result.last_mut().unwrap().push(*current.last().unwrap());
        }
    }
    result
}

pub(super) fn expand(points: Vec<Vec<Point2<f32>>>, len: f32) -> Vec<Vec<Point2<f32>>> {
    points
        .into_iter()
        .map(|mut v| {
            let temp = v
                .windows(3)
                .rev()
                .flat_map(|triple| {
                    let c = triple[1];
                    let d0 = (triple[0] - c).normalize();
                    let d1 = (triple[2] - c).normalize();
                    if d0.dot(&d1) < -0.8 {
                        vec![
                            c + len * normal(d1),
                            c - len * (d0 + d1).normalize(),
                            c - len * normal(d0),
                        ]
                    } else {
                        vec![c + (d0 + d1) * len / cross_numeric(d0, d1)]
                    }
                })
                .collect::<Vec<_>>();
            v.push(v[v.len() - 1] + len * normal(v[v.len() - 1] - v[v.len() - 2]).normalize());
            v.extend(temp);
            v.push(v[0] - len * normal(v[0] - v[1]).normalize());
            v
        })
        .collect()
}

/// 叉积
#[inline]
fn cross_numeric(v0: Vector2<f32>, v1: Vector2<f32>) -> f32 {
    v0[1] * v1[0] - v0[0] * v1[1]
}

/// 法向量
#[inline]
fn normal(v: Vector2<f32>) -> Vector2<f32> {
    vector(-v[1], v[0])
}
