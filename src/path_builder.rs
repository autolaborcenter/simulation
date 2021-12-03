use crate::{isometry, Isometry2, Vector2};

pub struct PathBuilder {
    ttl: usize,
    current: Isometry2<f32>,
    step: Vector2<f32>,
}

impl From<Isometry2<f32>> for PathBuilder {
    fn from(target: Isometry2<f32>) -> Self {
        let vec = target.translation.vector;
        let len = vec.norm();
        let step = 0.2 * vec / len;
        let ttl = (len / 0.2) as usize;
        Self {
            ttl,
            current: isometry(0.0, 0.0, step[0], step[1]),
            step,
        }
    }
}

impl Iterator for PathBuilder {
    type Item = Isometry2<f32>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.ttl > 0 {
            self.ttl -= 1;
            self.current.translation.vector += self.step;
            Some(self.current)
        } else {
            None
        }
    }
}
