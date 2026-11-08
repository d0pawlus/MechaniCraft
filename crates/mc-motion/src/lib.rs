//! Input motion laws for planar mechanism kinematics.
//!
//! An [`InputLaw`] is a sequence of [`Segment`]s played back to back. Each
//! segment starts where the previous one ended. Evaluation returns the
//! position `u`, velocity `v` and acceleration `a` of the input coordinate at
//! a given time.

/// One piece of an input motion law.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Segment {
    /// Constant-velocity ramp: moves by `displacement` over `duration`.
    ///
    /// `steps` is the number of equal time intervals used when building the
    /// time grid. A value of `0` is treated as `1`.
    Linear {
        duration: f64,
        steps: u32,
        displacement: f64,
    },
}

impl Segment {
    fn duration(&self) -> f64 {
        match *self {
            Segment::Linear { duration, .. } => duration,
        }
    }

    fn displacement(&self) -> f64 {
        match *self {
            Segment::Linear { displacement, .. } => displacement,
        }
    }

    fn steps(&self) -> u32 {
        match *self {
            Segment::Linear { steps, .. } => steps.max(1),
        }
    }
}

/// A motion law: segments played in order, starting at position `0.0`.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct InputLaw {
    pub segments: Vec<Segment>,
}

impl InputLaw {
    /// Total duration of all segments.
    pub fn duration(&self) -> f64 {
        self.segments.iter().map(Segment::duration).sum()
    }

    /// Evaluates the law at time `t`, returning `(u, v, a)`.
    ///
    /// - `t < 0` is clamped to the start: `u` is the start position and
    ///   `v = a = 0`.
    /// - At a segment boundary the segment that starts at `t` is used.
    /// - `t == duration()` returns the end position with the velocity of the
    ///   last segment and `a = 0`.
    /// - `t > duration()` returns the end position with `v = a = 0`.
    pub fn eval(&self, t: f64) -> (f64, f64, f64) {
        if t < 0.0 {
            return (0.0, 0.0, 0.0);
        }

        let mut t_start = 0.0;
        let mut u_start = 0.0;
        let mut last_velocity = 0.0;

        for segment in &self.segments {
            let duration = segment.duration();
            let displacement = segment.displacement();
            let t_end = t_start + duration;
            let velocity = if duration > 0.0 {
                displacement / duration
            } else {
                0.0
            };

            if t < t_end {
                let u = u_start + velocity * (t - t_start);
                return (u, velocity, 0.0);
            }

            t_start = t_end;
            u_start += displacement;
            last_velocity = velocity;
        }

        if t == t_start {
            (u_start, last_velocity, 0.0)
        } else {
            (u_start, 0.0, 0.0)
        }
    }

    /// Time grid for all segments.
    ///
    /// Each segment contributes `steps` equal intervals. Points at segment
    /// joints are shared, so the result has no duplicates. The first point is
    /// `0.0` and the last is `duration()`. An empty law gives `[0.0]`.
    pub fn time_grid(&self) -> Vec<f64> {
        let mut grid = vec![0.0];
        let mut t_start = 0.0;

        for segment in &self.segments {
            let duration = segment.duration();
            let steps = segment.steps();
            for i in 1..=steps {
                grid.push(t_start + duration * f64::from(i) / f64::from(steps));
            }
            t_start += duration;
        }

        grid
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use std::f64::consts::PI;

    const POS_TOL: f64 = 1e-9;
    const VEL_TOL: f64 = 1e-8;
    const ACC_TOL: f64 = 1e-6;

    fn two_segment_law() -> InputLaw {
        InputLaw {
            segments: vec![
                Segment::Linear {
                    duration: 1.0,
                    steps: 10,
                    displacement: 1.0,
                },
                Segment::Linear {
                    duration: 2.0,
                    steps: 10,
                    displacement: -1.0,
                },
            ],
        }
    }

    #[test]
    fn linear_eval() {
        let law = InputLaw {
            segments: vec![Segment::Linear {
                duration: 1.0,
                steps: 36,
                displacement: 2.0 * PI,
            }],
        };
        let (u, v, a) = law.eval(0.5);
        assert_abs_diff_eq!(u, PI, epsilon = POS_TOL);
        assert_abs_diff_eq!(v, 2.0 * PI, epsilon = VEL_TOL);
        assert_abs_diff_eq!(a, 0.0, epsilon = ACC_TOL);
    }

    #[test]
    fn segments_chain() {
        let law = two_segment_law();
        assert_abs_diff_eq!(law.eval(1.0).0, 1.0, epsilon = POS_TOL);
        assert_abs_diff_eq!(law.eval(3.0).0, 0.0, epsilon = POS_TOL);
        assert_abs_diff_eq!(law.eval(2.0).1, -0.5, epsilon = VEL_TOL);
    }

    #[test]
    fn time_grid_counts() {
        let grid = two_segment_law().time_grid();
        assert_eq!(grid.len(), 21);
        assert_abs_diff_eq!(grid[0], 0.0, epsilon = POS_TOL);
        assert_abs_diff_eq!(grid[20], 3.0, epsilon = POS_TOL);
        assert_abs_diff_eq!(grid[10], 1.0, epsilon = POS_TOL);
    }

    #[test]
    fn eval_clamps_beyond_end() {
        let (u, v, a) = two_segment_law().eval(5.0);
        assert_abs_diff_eq!(u, 0.0, epsilon = POS_TOL);
        assert_abs_diff_eq!(v, 0.0, epsilon = VEL_TOL);
        assert_abs_diff_eq!(a, 0.0, epsilon = ACC_TOL);
    }
}
