use super::cross_numeric;
use parry2d::na::Point2;

type Point = Point2<f32>;

/// 线段
pub(super) struct Segment(pub Point, pub Point);

impl Segment {
    pub fn len(&self) -> f32 {
        (self.0 - self.1).norm()
    }

    /// 两线段是否相交
    pub fn intersection(&self, others: &Segment) -> Option<f32> {
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

    pub fn ray_cast(&self, polygon: &[Point]) -> Option<f32> {
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
                    .reduce(|a, b| f32::min(a, b))
            }
        }
        .map(|t| self.len() * t)
    }
}

#[test]
fn test_intersection() {
    use crate::point;
    const S0: Segment = Segment(point(0.0, 0.0), point(1.0, 0.0));
    const S1: Segment = Segment(point(0.0, -1.0), point(1.0, 1.0));
    assert_eq!(Some(0.5), S0.intersection(&S1));
}
