use crate::{isometry, point, Isometry2, Point2};

pub struct PathBuilder {
    points: Vec<Point2<f32>>,
    coords: Isometry2<f32>,
    current: f32,
    step: f32,
    total: f32,
}

impl PathBuilder {
    #[inline]
    pub fn new(points: Vec<Point2<f32>>, light_radius: f32) -> Self {
        Self {
            points,
            coords: isometry(light_radius - 0.22, 0.0, 1.0, 0.0),
            current: light_radius,
            step: 0.2,
            total: 0.0,
        }
    }
}

impl Iterator for PathBuilder {
    type Item = Isometry2<f32>;

    fn next(&mut self) -> Option<Self::Item> {
        let current = self.current + self.step;
        if current < self.total {
            self.current = current;
            Some(self.coords * isometry(self.current, 0.0, 1.0, 0.0))
        } else if let Some(next) = self.points.pop() {
            let current = self.coords * point(self.total, 0.0);
            let target = next - current;

            self.current = 0.0;
            self.total = target.norm();

            let dir = target / self.total;
            self.coords = isometry(current[0], current[1], dir[0], dir[1]);

            Some(self.coords * isometry(self.current, 0.0, 1.0, 0.0))
        } else {
            None
        }
    }
}
