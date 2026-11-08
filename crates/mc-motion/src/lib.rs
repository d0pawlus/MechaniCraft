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
        self.segment_ends().last().copied().unwrap_or(0.0)
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

        let ends = self.segment_ends();
        let mut t_start = 0.0;
        let mut u_start = 0.0;
        let mut last_velocity = 0.0;

        for (segment, &t_end) in self.segments.iter().zip(&ends) {
            let duration = segment.duration();
            let displacement = segment.displacement();
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
            if duration > 0.0 {
                last_velocity = velocity;
            }
        }

        if t == t_start {
            (u_start, last_velocity, 0.0)
        } else {
            (u_start, 0.0, 0.0)
        }
    }

    /// Time grid for all segments.
    ///
    /// Each segment with a positive duration contributes `steps` equal
    /// intervals. Zero-duration segments contribute no points. Segment joints
    /// use exactly the same end times as [`InputLaw::eval`], so the grid is
    /// strictly increasing, starts at `0.0` and ends at [`InputLaw::duration`].
    /// An empty law gives `[0.0]`.
    pub fn time_grid(&self) -> Vec<f64> {
        let ends = self.segment_ends();
        let mut grid = vec![0.0];
        let mut t_start = 0.0;

        for (segment, &t_end) in self.segments.iter().zip(&ends) {
            let duration = segment.duration();
            if duration > 0.0 {
                let steps = segment.steps();
                for i in 1..steps {
                    let point = t_start + duration * f64::from(i) / f64::from(steps);
                    if point > grid[grid.len() - 1] && point < t_end {
                        grid.push(point);
                    }
                }
                if t_end > grid[grid.len() - 1] {
                    grid.push(t_end);
                }
            }
            t_start = t_end;
        }

        grid
    }

    /// End time of each segment, accumulated in order. Shared by `eval`,
    /// `duration` and `time_grid` so that all three agree bit for bit.
    fn segment_ends(&self) -> Vec<f64> {
        let mut ends = Vec::with_capacity(self.segments.len());
        let mut t = 0.0;
        for segment in &self.segments {
            t += segment.duration();
            ends.push(t);
        }
        ends
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

    #[test]
    fn eval_clamps_before_start() {
        assert_eq!(two_segment_law().eval(-1.0), (0.0, 0.0, 0.0));
    }

    #[test]
    fn grid_last_point_matches_eval_end_bitwise() {
        // d * n / n is not always d in IEEE doubles, so the last grid point
        // must be the same accumulated end time that eval and duration use.
        let law = InputLaw {
            segments: vec![Segment::Linear {
                duration: 0.1,
                steps: 3,
                displacement: 1.0,
            }],
        };
        let grid = law.time_grid();
        assert_eq!(*grid.last().unwrap(), law.duration());
        let (_, v, _) = law.eval(*grid.last().unwrap());
        assert_eq!(v, 1.0 / 0.1);
    }

    #[test]
    fn grid_joints_match_eval_segment_ends() {
        let law = two_segment_law();
        let ends = law.segment_ends();
        let grid = law.time_grid();
        assert_eq!(grid[10], ends[0]);
        assert_eq!(grid[20], ends[1]);
        assert_eq!(*grid.last().unwrap(), law.duration());
        // The joint belongs to the segment that starts there.
        assert_abs_diff_eq!(law.eval(1.0).1, -0.5, epsilon = VEL_TOL);
    }

    #[test]
    fn zero_duration_segment_adds_no_points() {
        let law = InputLaw {
            segments: vec![
                Segment::Linear {
                    duration: 0.0,
                    steps: 5,
                    displacement: 1.0,
                },
                Segment::Linear {
                    duration: 1.0,
                    steps: 2,
                    displacement: 1.0,
                },
            ],
        };
        let grid = law.time_grid();
        assert_eq!(grid, vec![0.0, 0.5, 1.0]);
    }
}
