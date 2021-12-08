use crate::Isometry2;
use path_tracking::Sector;

#[derive(Clone)]
pub struct Tracker<'a> {
    pub path: &'a path_tracking::Path,
    pub parameters: Parameters,
    pub index: (usize, usize),
    pub state: State,
}

#[derive(Clone, Copy)]
pub struct Parameters {
    pub search_range: Sector,    //
    pub light_radius: f32,       //
    pub r#loop: bool,            //
    pub auto_reinitialize: bool, //
}

#[derive(Clone, Copy, Debug)]
pub enum State {
    Relocating,   // 重新搜索
    Initializing, // 初始化
    Tracking,     // 连续循线
}

/// 跟踪失败信息
#[derive(Debug)]
pub enum Error {
    RelocationFailed, // 搜索失败
    OutOfPath,        // 丢失路径
    Complete,         // 任务完成
}

impl<'a> Tracker<'a> {
    /// 读取一条路径，检测其中的尖点
    pub fn new(path: &'a path_tracking::Path, parameters: Parameters) -> Self {
        Self {
            path,
            parameters,
            index: (0, 0),
            state: State::Relocating,
        }
    }

    /// 循线
    pub fn track(&mut self, pose: Isometry2<f32>) -> Result<(f32, f32), Error> {
        loop {
            match self.state {
                State::Relocating => {
                    if let Some(index) = self.path.relocate(path_tracking::RelocateConfig {
                        pose,
                        index: self.index,
                        light_radius: self.parameters.light_radius,
                        search_range: self.parameters.search_range,
                        r#loop: self.parameters.r#loop,
                    }) {
                        self.index = index;
                        self.state = State::Initializing;
                    } else {
                        return Err(Error::RelocationFailed);
                    }
                }
                State::Initializing => {
                    match path_tracking::track::goto(
                        pose.inv_mul(&self.path.slice(self.index)[0]),
                        self.parameters.light_radius,
                    ) {
                        Some(next) => return Ok(next),
                        None => self.state = State::Tracking,
                    }
                }
                State::Tracking => {
                    match self.path.promote(path_tracking::PromoteConfig {
                        pose,
                        index: self.index,
                        light_radius: self.parameters.light_radius,
                    }) {
                        Some(i) => {
                            self.index.1 = i;
                            let to_local = pose.inverse();
                            let rudder = path_tracking::track::track(
                                self.path.slice(self.index).iter().map(|p| to_local * p),
                                self.parameters.light_radius,
                            )
                            .unwrap();
                            return Ok((1.0, rudder));
                        }
                        None => {
                            if self.path.slice(self.index).len() < 2 {
                                self.state = if self.index.0 == self.path.0.len() - 1 {
                                    if !self.parameters.r#loop {
                                        return Err(Error::Complete);
                                    }
                                    self.index.0 = 0;
                                    self.index.1 = 0;
                                    State::Relocating
                                } else {
                                    self.index.0 += 1;
                                    self.index.1 = 0;
                                    State::Initializing
                                };
                            } else {
                                if self.parameters.auto_reinitialize {
                                    self.state = State::Initializing;
                                } else {
                                    return Err(Error::OutOfPath);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
