use crate::{isometry, vector, Isometry2, Point2, Vector2};
use std::cmp::Ordering;

mod outlines;
mod ray_cast;
mod simplify;

pub(super) use outlines::*;
pub(super) use ray_cast::ray_cast;
pub(super) use simplify::*;

/// 障碍物对象
#[derive(Clone)]
pub struct Obstacle {
    pub front: Vec<Polar>,
    pub back: Vec<Polar>,
}

pub enum ObstacleError {
    Empty,
    Besieged,
}

impl Obstacle {
    /// 构造障碍物对象
    pub fn new(
        sensor_on_robot: Isometry2<f32>,
        points: &[Point2<f32>],
        width: f32,
    ) -> Result<Self, ObstacleError> {
        if points.len() < 3 {
            Err(ObstacleError::Empty)
        } else {
            let mut vertex = enlarge(points, width * 0.5)
                .into_iter()
                .map(|p| Polar::from(sensor_on_robot * p));

            let head = vertex.next().unwrap();
            let mut last = head;

            let mut buffer = [
                Vec::<Polar>::new(),
                Vec::<Polar>::new(),
                Vec::<Polar>::new(),
            ];
            {
                let mut dir = Ordering::Equal;
                let mut step = 0;
                for p in vertex {
                    let next = p.theta.partial_cmp(&last.theta).unwrap();
                    if dir != Ordering::Equal && dir != next {
                        step += 1;
                        if step == 3 {
                            return Err(ObstacleError::Besieged);
                        }
                    }
                    last = p;
                    dir = next;
                    buffer[step].push(p);
                }
                let next = head.theta.partial_cmp(&last.theta).unwrap();
                if dir != Ordering::Equal && dir != next {
                    step += 1;
                    if step == 3 {
                        return Err(ObstacleError::Besieged);
                    }
                }
                buffer[step].push(head);
            }

            let first = std::mem::replace(unsafe { buffer.get_unchecked_mut(1) }, vec![]);
            let mut second = std::mem::replace(unsafe { buffer.get_unchecked_mut(2) }, vec![]);
            second.extend(buffer[0].iter());
            if first.last().unwrap().theta > second.last().unwrap().theta {
                Ok(Self {
                    front: first,
                    back: second,
                })
            } else {
                Ok(Self {
                    front: second,
                    back: first,
                })
            }
        }
    }

    /// 判断点是否在障碍物内
    #[inline]
    pub fn contains(&self, p: Point2<f32>) -> bool {
        self.check_relation(p.into()) == Some(Ordering::Equal)
    }

    /// 计算线段与障碍物交点，返回可能绕过障碍物的倒序的一列点
    pub fn go_through(
        &self,
        path: &mut impl Iterator<Item = (usize, Point2<f32>)>,
        trend: &mut f32,
    ) -> Polar {
        // 找到离开位置，如果可直达，直接去
        while let Some((_, p)) = path.next() {
            let polar = p.into();
            match self.check_relation(polar) {
                Some(Ordering::Less) | None => return polar,
                Some(Ordering::Greater) => break,
                Some(Ordering::Equal) => {}
            }
        }
        // 选需要转向更小的一边
        let left = self.front.last().unwrap();
        let right = self.back.last().unwrap();
        if left.theta.abs() * *trend < right.theta.abs() {
            *trend *= 0.998;
            *left
        } else {
            *trend *= 1.002;
            *right
        }
    }

    fn check_relation(&self, polar: Polar) -> Option<Ordering> {
        let left = self.front.last().unwrap();
        let right = self.back.last().unwrap();
        if polar.theta < right.theta || left.theta < polar.theta {
            return None;
        }
        let ord_front = match self
            .front
            .binary_search_by(|probe| probe.theta.partial_cmp(&polar.theta).unwrap())
        {
            Ok(i) => polar.rho.partial_cmp(&self.front[i].rho).unwrap(),
            Err(i) => {
                let right = if i == 0 { *right } else { self.front[i - 1] };
                cmp(polar, (right, self.front[i]))
            }
        };
        if ord_front != Ordering::Greater {
            return Some(ord_front);
        }
        let ord_back = match self
            .back
            .binary_search_by(|probe| polar.theta.partial_cmp(&probe.theta).unwrap())
        {
            Ok(i) => polar.rho.partial_cmp(&self.back[i].rho).unwrap(),
            Err(i) => {
                let left = if i == 0 { *left } else { self.back[i - 1] };
                cmp(polar, (self.back[i], left))
            }
        };
        return Some(if ord_back != Ordering::Greater {
            Ordering::Equal
        } else {
            Ordering::Greater
        });
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Polar {
    pub rho: f32,
    pub theta: f32,
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
    let mut result = Vec::with_capacity(v.len());
    result.push(v[0] + len * normal((v[1] - v[0]).normalize()));
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

#[inline]
fn cmp(c: Polar, seg: (Polar, Polar)) -> Ordering {
    let a = seg.0.to_point();
    let b = seg.1.to_point();
    let c = c.to_point();
    cross_numeric(b - a, c - b).partial_cmp(&0.0).unwrap()
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
