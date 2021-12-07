use crate::{isometry, point, Isometry2, Point2};

pub struct PathBuilder {
    points: Vec<Point2<f32>>,
    coords: Isometry2<f32>,
    current: f32,
    step: f32,
    total: f32,
}

impl PathBuilder {
    pub fn new(mut points: Vec<Point2<f32>>) -> Self {
        let target = points.pop().unwrap();
        let total = target.coords.norm();
        let dir = target.coords / total;
        Self {
            points,
            coords: isometry(0.0, 0.0, dir[0], dir[1]),
            current: 0.0,
            step: 0.2,
            total,
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
            let current = self.coords * point(self.current, 0.0);
            let target = next - current;
            self.current = 0.0;
            self.total = target.norm();
            let dir = target / self.total;
            self.coords = isometry(current[0], current[0], dir[0], dir[1]);
            None
        } else {
            None
        }
    }
}
