//! Animation clock over the time grid of a trajectory.

/// Current time on a timeline of frame times, with play, speed and loop.
#[derive(Debug, Clone, PartialEq)]
pub struct Playback {
    times: Vec<f64>,
    /// Current time; may lie between frames.
    pub time: f64,
    pub playing: bool,
    /// Model seconds per wall-clock second.
    pub speed: f64,
    /// Restart from the beginning at the end instead of stopping.
    pub looping: bool,
}

impl Playback {
    /// Paused playback at the first frame. `times` must be ascending.
    pub fn new(times: Vec<f64>) -> Self {
        let time = times.first().copied().unwrap_or(0.0);
        Self {
            times,
            time,
            playing: false,
            speed: 1.0,
            looping: true,
        }
    }

    /// First and last frame time, if there are frames.
    pub fn range(&self) -> Option<(f64, f64)> {
        Some((*self.times.first()?, *self.times.last()?))
    }

    /// Moves the clock by `dt` wall-clock seconds when playing.
    pub fn advance(&mut self, dt: f64) {
        let Some((start, end)) = self.range() else {
            return;
        };
        if !self.playing || !dt.is_finite() {
            return;
        }
        let length = end - start;
        let t = self.time + dt * self.speed;
        if t <= end {
            self.time = t.max(start);
        } else if self.looping && length > 0.0 {
            self.time = start + (t - start).rem_euclid(length);
        } else {
            self.time = end;
            self.playing = false;
        }
    }

    /// Index of the last frame whose time is not after the current time
    /// (the first frame before the start); `None` without frames.
    pub fn frame(&self) -> Option<usize> {
        if self.times.is_empty() {
            return None;
        }
        let after = self.times.partition_point(|&t| t <= self.time);
        Some(after.saturating_sub(1))
    }

    /// Jumps to frame `index`, clamped to the last frame.
    pub fn seek_frame(&mut self, index: usize) {
        if let Some(&t) = self.times.get(index).or(self.times.last()) {
            self.time = t;
        }
    }

    /// Number of frames.
    pub fn frame_count(&self) -> usize {
        self.times.len()
    }
}
