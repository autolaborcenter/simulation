use std::time::{Duration, Instant};

pub(super) struct Ticker {
    period: Duration,
    time: Instant,
}

impl Ticker {
    #[inline]
    pub fn new(period: Duration) -> Self {
        Self {
            period,
            time: Instant::now(),
        }
    }

    #[inline]
    pub async fn wait(&mut self) {
        self.time += self.period;
        if let Some(dur) = self.time.checked_duration_since(Instant::now()) {
            async_std::task::sleep(dur).await;
        }
    }
}
