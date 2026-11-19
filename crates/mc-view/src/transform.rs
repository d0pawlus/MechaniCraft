//! Mapping between model coordinates (metres, y up) and screen pixels (y down).

use crate::scene::Bounds;

/// Smallest model extent used for fitting, so a single point or a straight
/// horizontal mechanism still gets a finite scale.
const MIN_EXTENT: f64 = 1e-3;

/// Uniform scale plus translation, with the y axis flipped.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ViewTransform {
    /// Pixels per model unit.
    pub scale: f64,
    /// Model point shown at the centre of the screen rectangle.
    center_model: [f64; 2],
    center_screen: [f64; 2],
}

impl ViewTransform {
    /// Largest uniform scale that shows `bounds` inside the screen rectangle
    /// at `origin` with `size`, leaving `margin` pixels on each side, centred.
    pub fn fit(bounds: Bounds, origin: [f64; 2], size: [f64; 2], margin: f64) -> Self {
        let extent = |i: usize| (bounds.max[i] - bounds.min[i]).max(MIN_EXTENT);
        let avail = |i: usize| (size[i] - 2.0 * margin).max(1.0);
        let scale = (avail(0) / extent(0)).min(avail(1) / extent(1));
        let mid = |i: usize| 0.5 * (bounds.min[i] + bounds.max[i]);
        Self {
            scale,
            center_model: [mid(0), mid(1)],
            center_screen: [origin[0] + 0.5 * size[0], origin[1] + 0.5 * size[1]],
        }
    }

    /// Screen position of model point `p`.
    pub fn to_screen(&self, p: [f64; 2]) -> [f64; 2] {
        [
            self.center_screen[0] + (p[0] - self.center_model[0]) * self.scale,
            self.center_screen[1] - (p[1] - self.center_model[1]) * self.scale,
        ]
    }
}
