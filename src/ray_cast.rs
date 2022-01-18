use crate::{Isometry2, Point2, Polar, Sector, Vector2};
use rand::{thread_rng, Rng};
use std::f32::consts::PI;

/// 相邻射线夹角
const STEP: f32 = PI / 360.0;

/// 放出射线生成传感器坐标系上的点云
pub(crate) fn ray_cast(
    robot_on_world: Isometry2<f32>,
    sensor_on_robot: Isometry2<f32>,
    obstacles: impl IntoIterator<Item = Vec<Point2<f32>>>,
    range: Sector,
) -> Vec<Polar> {
    let pose = robot_on_world * sensor_on_robot;
    // 转本地多边形障碍物
    // 只考虑至少一个点在距离机器人小于 10 米的
    let obstacles: Vec<Vec<_>> = {
        let center = pose.translation.vector;
        let to_local = pose.inverse();
        obstacles
            .into_iter()
            .filter(|v| v.iter().any(|p| (p.coords - center).norm_squared() < 100.0))
            .map(|v| v.iter().map(|p| to_local * p).collect())
            .collect()
    };
    //
    let dir_range = range.angle * 0.5;
    let mut result = Vec::new();
    let mut theta = -dir_range;
    while theta <= dir_range {
        let ray = Segment(
            Polar { rho: 0.2, theta }.into(),
            Polar {
                rho: range.radius,
                theta,
            }
            .into(),
        );
        theta += STEP;
        let _ = obstacles
            .iter()
            .filter_map(|c| ray.ray_cast(c))
            .min_by(|a, b| a.partial_cmp(b).unwrap())
            .map(|rho| rho + 0.2 + thread_rng().gen_range(-0.01..0.01))
            .map(|rho| result.push(Polar { rho, theta }));
    }
    result
}

/// 线段
struct Segment(pub Point2<f32>, pub Point2<f32>);

impl Segment {
    #[inline]
    pub fn len(&self) -> f32 {
        (self.0 - self.1).norm()
    }

    /// 两线段是否相交
    pub fn intersection(&self, others: &Segment) -> Option<f32> {
        /// 叉积 === 两向量围成平行四边形面积
        #[inline]
        fn cross_numeric(v0: Vector2<f32>, v1: Vector2<f32>) -> f32 {
            v0[1] * v1[0] - v0[0] * v1[1]
        }

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
    pub fn ray_cast(&self, polygon: &[Point2<f32>]) -> Option<f32> {
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
                    .reduce(f32::min)
            }
        }
        .map(|t| self.len() * t)
    }
}

#[test]
fn test_intersection() {
    const S0: Segment = Segment(point!(0.0, 0.0), point!(1.0, 0.0));
    const S1: Segment = Segment(point!(0.0, -1.0), point!(1.0, 1.0));
    assert_eq!(Some(0.5), S0.intersection(&S1));
}
