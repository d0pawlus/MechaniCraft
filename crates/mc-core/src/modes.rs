//! Element deformation modes `e(q)` with gradients and convective terms.
//!
//! Every mode is measured relative to the initial configuration
//! (`Coord::value0`), so all modes vanish at `q = value0`. The Newton solver
//! uses `value` as residual, `grad` as Jacobian row and `conv` as the
//! convective term of the acceleration equation.

use std::f64::consts::PI;

use crate::model::{CoordId, Element, ElementKind, Mechanism};

/// How a deformation mode is constrained by the mechanism definition.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModeRole {
    /// The mode is held at zero.
    Rigid,
    /// The mode is unconstrained (a degree of freedom).
    Free,
    /// The mode follows `mech.inputs[i]`.
    Driven(usize),
}

/// Value, gradient and convective term of one deformation mode.
#[derive(Debug, Clone, PartialEq)]
pub struct ModeEval {
    /// Mode value `e(q)`.
    pub value: f64,
    /// Partial derivatives `∂e/∂q_k` as `(coordinate, partial)`, fixed
    /// coordinates included, duplicates combined.
    pub grad: Vec<(CoordId, f64)>,
    /// Convective term `q̇ᵀ (∂²e/∂q²) q̇`.
    pub conv: f64,
}

/// Maps an angle to (−π, π].
fn wrap(x: f64) -> f64 {
    let r = x.rem_euclid(2.0 * PI);
    if r > PI { r - 2.0 * PI } else { r }
}

fn add_grad(grad: &mut Vec<(CoordId, f64)>, id: CoordId, v: f64) {
    if let Some(entry) = grad.iter_mut().find(|(c, _)| *c == id) {
        entry.1 += v;
    } else {
        grad.push((id, v));
    }
}

/// Geometry of a two-node element: `d = q − p`, its length and angle.
struct Span {
    d: [f64; 2],
    dd: [f64; 2],
    len: f64,
    alpha: f64,
    len0: f64,
    alpha0: f64,
}

impl Span {
    fn new(mech: &Mechanism, p: [CoordId; 2], q: [CoordId; 2], x: &[f64], xd: &[f64]) -> Self {
        let v0 = |id: CoordId| mech.coords[id.0].value0;
        let d0 = [v0(q[0]) - v0(p[0]), v0(q[1]) - v0(p[1])];
        let d = [x[q[0].0] - x[p[0].0], x[q[1].0] - x[p[1].0]];
        let dd = [xd[q[0].0] - xd[p[0].0], xd[q[1].0] - xd[p[1].0]];
        Span {
            d,
            dd,
            len: d[0].hypot(d[1]),
            alpha: d[1].atan2(d[0]),
            len0: d0[0].hypot(d0[1]),
            alpha0: d0[1].atan2(d0[0]),
        }
    }

    /// Gradient of `L` with respect to the four node coordinates.
    fn grad_len(&self, p: [CoordId; 2], q: [CoordId; 2], scale: f64, g: &mut Vec<(CoordId, f64)>) {
        let (ux, uy) = (self.d[0] / self.len, self.d[1] / self.len);
        add_grad(g, q[0], scale * ux);
        add_grad(g, q[1], scale * uy);
        add_grad(g, p[0], -scale * ux);
        add_grad(g, p[1], -scale * uy);
    }

    /// Gradient of `α` with respect to the four node coordinates.
    fn grad_alpha(&self, p: [CoordId; 2], q: [CoordId; 2], scale: f64, g: &mut Vec<(CoordId, f64)>) {
        let l2 = self.len * self.len;
        let (ax, ay) = (-self.d[1] / l2, self.d[0] / l2);
        add_grad(g, q[0], scale * ax);
        add_grad(g, q[1], scale * ay);
        add_grad(g, p[0], -scale * ax);
        add_grad(g, p[1], -scale * ay);
    }

    fn conv_len(&self) -> f64 {
        let dot = self.d[0] * self.dd[0] + self.d[1] * self.dd[1];
        let dd2 = self.dd[0] * self.dd[0] + self.dd[1] * self.dd[1];
        (dd2 - dot * dot / (self.len * self.len)) / self.len
    }

    fn conv_alpha(&self) -> f64 {
        let dot = self.d[0] * self.dd[0] + self.d[1] * self.dd[1];
        let cross = self.d[0] * self.dd[1] - self.d[1] * self.dd[0];
        let l2 = self.len * self.len;
        -2.0 * cross * dot / (l2 * l2)
    }
}

fn length_mode(span: &Span, p: [CoordId; 2], q: [CoordId; 2]) -> ModeEval {
    let mut grad = Vec::with_capacity(4);
    span.grad_len(p, q, 1.0, &mut grad);
    ModeEval {
        value: span.len - span.len0,
        grad,
        conv: span.conv_len(),
    }
}

/// Bending mode `wrap((φ−φ0) − (α−α0))` for angle coordinate `phi`.
fn bending_mode(
    mech: &Mechanism,
    span: &Span,
    p: [CoordId; 2],
    q: [CoordId; 2],
    phi: CoordId,
    x: &[f64],
) -> ModeEval {
    let mut grad = Vec::with_capacity(5);
    add_grad(&mut grad, phi, 1.0);
    span.grad_alpha(p, q, -1.0, &mut grad);
    ModeEval {
        value: wrap((x[phi.0] - mech.coords[phi.0].value0) - (span.alpha - span.alpha0)),
        grad,
        conv: -span.conv_alpha(),
    }
}

fn rotation_mode(mech: &Mechanism, a: CoordId, b: CoordId, x: &[f64]) -> ModeEval {
    let da = x[a.0] - mech.coords[a.0].value0;
    let db = x[b.0] - mech.coords[b.0].value0;
    let mut grad = Vec::with_capacity(2);
    add_grad(&mut grad, b, 1.0);
    add_grad(&mut grad, a, -1.0);
    // No wrap: motors and hinges rotate continuously and without bound.
    ModeEval {
        value: db - da,
        grad,
        conv: 0.0,
    }
}

/// Evaluates all deformation modes of `el` at position `q` and velocity `qd`.
///
/// Beams and sliders return three modes (length, bending at node 1, bending
/// at node 2); hinges and rotational motors return one. `q` and `qd` are
/// indexed by [`CoordId`] and must satisfy `q.len() == qd.len() ==
/// mech.coords.len()`; [`crate::analyze`] validates the model but this
/// function does not check lengths.
pub fn element_modes(mech: &Mechanism, el: &Element, q: &[f64], qd: &[f64]) -> Vec<ModeEval> {
    match &el.kind {
        ElementKind::Beam { p, q: qn, a, b } | ElementKind::Slider { p, q: qn, a, b } => {
            let span = Span::new(mech, *p, *qn, q, qd);
            vec![
                length_mode(&span, *p, *qn),
                bending_mode(mech, &span, *p, *qn, *a, q),
                bending_mode(mech, &span, *p, *qn, *b, q),
            ]
        }
        ElementKind::Hinge { a, b, .. } | ElementKind::RotMotor { a, b, .. } => {
            vec![rotation_mode(mech, *a, *b, q)]
        }
    }
}

/// Absolute angle of the line through the nodes of a beam or slider.
///
/// `value = atan2(d)` (not wrapped relative to the initial angle), `grad` is
/// the gradient of the angle over the four node coordinates and `conv` is
/// `qd^T (d2 alpha / dq2) qd`. Returns `None` for hinges and rotational
/// motors, which have no line.
pub fn line_angle(mech: &Mechanism, el: &Element, q: &[f64], qd: &[f64]) -> Option<ModeEval> {
    match &el.kind {
        ElementKind::Beam { p, q: qn, .. } | ElementKind::Slider { p, q: qn, .. } => {
            let span = Span::new(mech, *p, *qn, q, qd);
            let mut grad = Vec::with_capacity(4);
            span.grad_alpha(*p, *qn, 1.0, &mut grad);
            Some(ModeEval {
                value: span.alpha,
                grad,
                conv: span.conv_alpha(),
            })
        }
        ElementKind::Hinge { .. } | ElementKind::RotMotor { .. } => None,
    }
}

/// Lists the role of every mode as `(element index, mode index, role)`, in
/// element order and then mode order.
///
/// Beam and slider mode 0 (length change) is `Driven(i)` when `mech.inputs[i]`
/// targets the element; otherwise a slider leaves it `Free` and a beam holds it
/// `Rigid`. Hinge and motor mode 0: an input takes precedence over
/// `fixed` (`Driven(i)`), then `fixed` gives `Rigid`, else `Free`. All
/// remaining beam and slider modes are `Rigid`.
pub fn mode_roles(mech: &Mechanism) -> Vec<(usize, usize, ModeRole)> {
    let mut roles = Vec::new();
    for (idx, el) in mech.elements.iter().enumerate() {
        let input = mech.inputs.iter().position(|inp| inp.element_nr == el.nr);
        match &el.kind {
            ElementKind::Beam { .. } => {
                let first = input.map_or(ModeRole::Rigid, ModeRole::Driven);
                roles.push((idx, 0, first));
                roles.push((idx, 1, ModeRole::Rigid));
                roles.push((idx, 2, ModeRole::Rigid));
            }
            ElementKind::Slider { .. } => {
                let first = input.map_or(ModeRole::Free, ModeRole::Driven);
                roles.push((idx, 0, first));
                roles.push((idx, 1, ModeRole::Rigid));
                roles.push((idx, 2, ModeRole::Rigid));
            }
            ElementKind::Hinge { fixed, .. } | ElementKind::RotMotor { fixed, .. } => {
                let role = match (input, *fixed) {
                    (Some(i), _) => ModeRole::Driven(i),
                    (None, true) => ModeRole::Rigid,
                    (None, false) => ModeRole::Free,
                };
                roles.push((idx, 0, role));
            }
        }
    }
    roles
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::MechanismBuilder;
    use mc_motion::{InputLaw, Segment};

    fn law() -> InputLaw {
        InputLaw {
            segments: vec![Segment::Linear {
                duration: 1.0,
                steps: 10,
                displacement: 2.0 * PI,
            }],
        }
    }

    /// Slider-crank plus a slider and a hinge so every element kind is covered.
    fn mechanism() -> Mechanism {
        let mut b = MechanismBuilder::new();
        b.node(1, 0.0, 0.0, true, true)
            .node(2, 0.1, 0.0, false, false)
            .node(3, 0.4, 0.0, false, true)
            .angle(1001, 0.0, false)
            .angle(1002, 0.0, false)
            .angle(1003, 0.0, false)
            .angle(1004, 0.0, false)
            .angle(1005, 0.0, true)
            .beam(1, 1, 2, 1001, 1002)
            .beam(2, 2, 3, 1003, 1004)
            .rot_motor(3, 1005, 1001, false)
            .input(3, law());
        b.build().unwrap()
    }

    fn extended() -> Mechanism {
        let mut b = MechanismBuilder::new();
        b.node(1, 0.0, 0.0, true, true)
            .node(2, 0.1, 0.2, false, false)
            .node(3, 0.4, 0.1, false, false)
            .angle(1, 0.3, false)
            .angle(2, -0.2, false)
            .angle(3, 0.1, false)
            .angle(4, 0.5, false)
            .beam(1, 1, 2, 1, 2)
            .slider(2, 2, 3, 3, 4)
            .hinge(3, 2, 3, false);
        b.build().unwrap()
    }

    /// Tiny deterministic LCG returning values in [-1, 1].
    struct Lcg(u64);
    impl Lcg {
        fn next(&mut self) -> f64 {
            self.0 = self
                .0
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((self.0 >> 11) as f64 / (1u64 << 53) as f64) * 2.0 - 1.0
        }
    }

    fn value0(mech: &Mechanism) -> Vec<f64> {
        mech.coords.iter().map(|c| c.value0).collect()
    }

    fn perturbed(mech: &Mechanism, rng: &mut Lcg) -> Vec<f64> {
        value0(mech).into_iter().map(|v| v + 0.05 * rng.next()).collect()
    }

    fn all_values(mech: &Mechanism, q: &[f64], qd: &[f64]) -> Vec<f64> {
        mech.elements
            .iter()
            .flat_map(|el| element_modes(mech, el, q, qd))
            .map(|m| m.value)
            .collect()
    }

    #[test]
    fn modes_zero_at_initial() {
        for mech in [mechanism(), extended()] {
            let q = value0(&mech);
            let qd = vec![0.0; q.len()];
            let vals = all_values(&mech, &q, &qd);
            assert!(!vals.is_empty());
            for v in vals {
                assert_eq!(v, 0.0);
            }
        }
    }

    #[test]
    fn gradients_match_finite_difference() {
        let h = 1e-7;
        let mut rng = Lcg(42);
        for mech in [mechanism(), extended()] {
            let q = perturbed(&mech, &mut rng);
            let qd = vec![0.0; q.len()];
            for el in &mech.elements {
                let modes = element_modes(&mech, el, &q, &qd);
                for (mi, mode) in modes.iter().enumerate() {
                    for k in 0..q.len() {
                        let eval_at = |delta: f64| {
                            let mut qq = q.clone();
                            qq[k] += delta;
                            element_modes(&mech, el, &qq, &qd)[mi].value
                        };
                        let fd = (eval_at(h) - eval_at(-h)) / (2.0 * h);
                        let analytic: f64 = mode
                            .grad
                            .iter()
                            .filter(|(c, _)| c.0 == k)
                            .map(|(_, g)| g)
                            .sum();
                        assert!(
                            (fd - analytic).abs() < 1e-6,
                            "element {} mode {} coord {}: fd {} vs {}",
                            el.nr,
                            mi,
                            k,
                            fd,
                            analytic
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn conv_matches_second_difference() {
        let h = 1e-4;
        let mut rng = Lcg(7);
        for mech in [mechanism(), extended()] {
            let q = perturbed(&mech, &mut rng);
            let qd: Vec<f64> = (0..q.len()).map(|_| rng.next()).collect();
            let at = |s: f64| -> Vec<f64> {
                let qq: Vec<f64> = q.iter().zip(&qd).map(|(a, v)| a + s * v).collect();
                all_values(&mech, &qq, &qd)
            };
            let (plus, mid, minus) = (at(h), at(0.0), at(-h));
            let convs: Vec<f64> = mech
                .elements
                .iter()
                .flat_map(|el| element_modes(&mech, el, &q, &qd))
                .map(|m| m.conv)
                .collect();
            for i in 0..convs.len() {
                let second = (plus[i] - 2.0 * mid[i] + minus[i]) / (h * h);
                assert!(
                    (second - convs[i]).abs() < 1e-4,
                    "mode {}: second diff {} vs conv {}",
                    i,
                    second,
                    convs[i]
                );
            }
        }
    }

    #[test]
    fn bending_mode_wraps() {
        let mut b = MechanismBuilder::new();
        b.node(1, 0.0, 0.0, true, true)
            .node(2, 0.1, 0.0, false, false)
            .angle(1, 0.0, false)
            .angle(2, 0.0, false)
            .beam(1, 1, 2, 1, 2);
        let mech = b.build().unwrap();
        let (x2, y2) = mech.node(2).unwrap();
        let mut q = value0(&mech);
        q[x2.0] = 0.1 * 3.0_f64.cos();
        q[y2.0] = 0.1 * 3.0_f64.sin();
        q[mech.angle(1).unwrap().0] = 3.0 - 2.0 * PI;
        let qd = vec![0.0; q.len()];
        let modes = element_modes(&mech, &mech.elements[0], &q, &qd);
        assert!(modes[1].value.abs() < 1e-12, "got {}", modes[1].value);
    }

    #[test]
    fn line_angle_matches_bending_mode() {
        let mut rng = Lcg(11);
        for mech in [mechanism(), extended()] {
            let q = perturbed(&mech, &mut rng);
            let qd: Vec<f64> = (0..q.len()).map(|_| rng.next()).collect();
            for el in &mech.elements {
                let la = line_angle(&mech, el, &q, &qd);
                match &el.kind {
                    ElementKind::Hinge { .. } | ElementKind::RotMotor { .. } => {
                        assert!(la.is_none());
                    }
                    ElementKind::Beam { p, q: qn, .. } | ElementKind::Slider { p, q: qn, .. } => {
                        let la = la.unwrap();
                        let bend = &element_modes(&mech, el, &q, &qd)[1];
                        let d = [q[qn[0].0] - q[p[0].0], q[qn[1].0] - q[p[1].0]];
                        assert_eq!(la.value, d[1].atan2(d[0]));
                        for c in [p[0], p[1], qn[0], qn[1]] {
                            let g = |m: &ModeEval| -> f64 {
                                m.grad.iter().filter(|(k, _)| *k == c).map(|(_, v)| v).sum()
                            };
                            assert!((g(&la) + g(bend)).abs() < 1e-15);
                        }
                        assert!((la.conv + bend.conv).abs() < 1e-15);
                    }
                }
            }
        }
    }

    #[test]
    fn roles_for_slider_crank() {
        let mech = mechanism();
        let roles = mode_roles(&mech);
        assert_eq!(roles.len(), 7);
        let rot = roles.iter().find(|(e, _, _)| *e == 2).unwrap();
        assert_eq!(*rot, (2, 0, ModeRole::Driven(0)));
        let rigid = roles
            .iter()
            .filter(|(e, _, r)| *e < 2 && *r == ModeRole::Rigid)
            .count();
        assert_eq!(rigid, 6);
    }

    #[test]
    fn roles_for_driven_beam() {
        let mut b = MechanismBuilder::new();
        b.node(1, 0.0, 0.0, true, true)
            .node(2, 1.0, 0.0, false, true)
            .angle(1001, 0.0, false)
            .angle(1002, 0.0, false)
            .beam(1, 1, 2, 1001, 1002)
            .input(1, law());
        let mech = b.build().unwrap();
        assert_eq!(
            mode_roles(&mech),
            vec![
                (0, 0, ModeRole::Driven(0)),
                (0, 1, ModeRole::Rigid),
                (0, 2, ModeRole::Rigid),
            ]
        );
    }
}
