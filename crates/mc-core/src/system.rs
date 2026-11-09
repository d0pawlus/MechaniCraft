//! Shared assembly of the rigid/driven mode system used by kinematics and
//! kinetostatics.

use nalgebra::{DMatrix, DVector};

use crate::model::Mechanism;
use crate::modes::{ModeEval, ModeRole, element_modes, mode_roles};
use crate::solver::SolveError;

/// One equation: a mode that is rigid or driven.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Equation {
    /// Index into `mech.elements`.
    pub element: usize,
    /// Mode index within the element.
    pub mode: usize,
    /// `Some(i)` when driven by `mech.inputs[i]`, `None` when held at zero.
    pub input: Option<usize>,
}

/// Mode values, Jacobian over the free coordinates and convective terms.
#[derive(Debug, Clone, PartialEq)]
pub struct Assembly {
    pub value: DVector<f64>,
    /// `equations x free` matrix.
    pub jacobian: DMatrix<f64>,
    pub conv: DVector<f64>,
}

/// The square system of rigid and driven modes over the free coordinates.
#[derive(Debug, Clone)]
pub struct ModeSystem<'a> {
    mech: &'a Mechanism,
    equations: Vec<Equation>,
    /// Column of each coordinate in the Jacobian, `None` for fixed ones.
    column: Vec<Option<usize>>,
    free: Vec<usize>,
}

impl<'a> ModeSystem<'a> {
    /// Validates `mech` and builds the equation list (role order) and the free
    /// coordinate columns (ascending `CoordId`). Does not require inputs.
    pub fn new(mech: &'a Mechanism) -> Result<Self, SolveError> {
        mech.validate().map_err(SolveError::InvalidModel)?;
        let equations: Vec<Equation> = mode_roles(mech)
            .into_iter()
            .filter_map(|(element, mode, role)| match role {
                ModeRole::Free => None,
                ModeRole::Rigid => Some(Equation {
                    element,
                    mode,
                    input: None,
                }),
                ModeRole::Driven(i) => Some(Equation {
                    element,
                    mode,
                    input: Some(i),
                }),
            })
            .collect();
        let free: Vec<usize> = (0..mech.coords.len())
            .filter(|&i| !mech.coords[i].fixed)
            .collect();
        if equations.len() != free.len() {
            return Err(SolveError::NotSquare {
                equations: equations.len(),
                unknowns: free.len(),
            });
        }
        let mut column = vec![None; mech.coords.len()];
        for (col, &coord) in free.iter().enumerate() {
            column[coord] = Some(col);
        }
        Ok(ModeSystem {
            mech,
            equations,
            column,
            free,
        })
    }

    pub fn mechanism(&self) -> &'a Mechanism {
        self.mech
    }

    pub fn equations(&self) -> &[Equation] {
        &self.equations
    }

    /// `CoordId` indices of the free coordinates, in Jacobian column order.
    pub fn free(&self) -> &[usize] {
        &self.free
    }

    /// Targets `(u, v, a)` of every equation at time `t` (0 for rigid modes).
    pub fn targets(&self, t: f64) -> [DVector<f64>; 3] {
        let n = self.equations.len();
        let mut out = [DVector::zeros(n), DVector::zeros(n), DVector::zeros(n)];
        for (row, eq) in self.equations.iter().enumerate() {
            if let Some(i) = eq.input {
                let (u, v, a) = self.mech.inputs[i].law.eval(t);
                out[0][row] = u;
                out[1][row] = v;
                out[2][row] = a;
            }
        }
        out
    }

    /// Calls `f(row, mode)` for every equation, evaluating each element once.
    fn for_each_mode(&self, q: &[f64], qd: &[f64], mut f: impl FnMut(usize, &ModeEval)) {
        // Equations are ordered by element, so each element is evaluated once.
        let mut current: Option<(usize, Vec<ModeEval>)> = None;
        for (row, eq) in self.equations.iter().enumerate() {
            if current.as_ref().is_none_or(|(el, _)| *el != eq.element) {
                let modes = element_modes(self.mech, &self.mech.elements[eq.element], q, qd);
                current = Some((eq.element, modes));
            }
            if let Some((_, modes)) = &current {
                f(row, &modes[eq.mode]);
            }
        }
    }

    /// Evaluates the modes at position `q` and velocity `qd`.
    pub fn assemble(&self, q: &[f64], qd: &[f64]) -> Assembly {
        let n = self.equations.len();
        let mut a = Assembly {
            value: DVector::zeros(n),
            jacobian: DMatrix::zeros(n, self.free.len()),
            conv: DVector::zeros(n),
        };
        self.for_each_mode(q, qd, |row, mode| {
            a.value[row] = mode.value;
            a.conv[row] = mode.conv;
            for &(c, g) in &mode.grad {
                if let Some(col) = self.column[c.0] {
                    a.jacobian[(row, col)] += g;
                }
            }
        });
        a
    }

    /// Jacobian over all coordinates (fixed ones included), `equations x coords`.
    pub fn full_jacobian(&self, q: &[f64]) -> DMatrix<f64> {
        let zeros = vec![0.0; q.len()];
        let mut j = DMatrix::zeros(self.equations.len(), self.mech.coords.len());
        self.for_each_mode(q, &zeros, |row, mode| {
            for &(c, g) in &mode.grad {
                j[(row, c.0)] += g;
            }
        });
        j
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{CoordId, ElementKind, Mechanism, MechanismBuilder, ModelError};
    use crate::solver::SolveError;
    use mc_motion::{InputLaw, Segment};

    fn law() -> InputLaw {
        InputLaw {
            segments: vec![Segment::Linear {
                duration: 1.0,
                steps: 4,
                displacement: 1.0,
            }],
        }
    }

    fn crank(fix_y3: bool) -> Mechanism {
        let mut b = MechanismBuilder::new();
        b.node(1, 0.0, 0.0, true, true)
            .node(2, 0.1, 0.0, false, false)
            .node(3, 0.4, 0.0, false, fix_y3)
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

    fn perturbed(mech: &Mechanism) -> Vec<f64> {
        mech.coords
            .iter()
            .map(|c| if c.fixed { c.value0 } else { c.value0 + 0.01 })
            .collect()
    }

    #[test]
    fn mode_system_counts() {
        let mech = crank(true);
        let sys = ModeSystem::new(&mech).unwrap();
        assert_eq!(sys.equations().len(), 7);
        assert_eq!(sys.free().len(), 7);
        let driven: Vec<_> = sys.equations().iter().filter(|e| e.input.is_some()).collect();
        assert_eq!(driven.len(), 1);
        assert_eq!(driven[0].input, Some(0));
    }

    #[test]
    fn full_jacobian_restricted_to_free_equals_assemble() {
        let mech = crank(true);
        let sys = ModeSystem::new(&mech).unwrap();
        let zeros = vec![0.0; mech.coords.len()];
        let v0: Vec<f64> = mech.coords.iter().map(|c| c.value0).collect();
        for q in [v0, perturbed(&mech)] {
            let full = sys.full_jacobian(&q);
            let restricted = full.select_columns(sys.free());
            assert_eq!(restricted, sys.assemble(&q, &zeros).jacobian);
        }
    }

    #[test]
    fn full_jacobian_includes_fixed_columns() {
        let mech = crank(true);
        let sys = ModeSystem::new(&mech).unwrap();
        let q: Vec<f64> = mech.coords.iter().map(|c| c.value0).collect();
        let full = sys.full_jacobian(&q);
        assert_eq!((full.nrows(), full.ncols()), (7, 11));
        let row = sys
            .equations()
            .iter()
            .position(|e| e.element == 0 && e.mode == 0)
            .unwrap();
        let (x1, _) = mech.node(1).unwrap();
        assert!(mech.coords[x1.0].fixed);
        assert!((full[(row, x1.0)] + 1.0).abs() < 1e-15);
    }

    #[test]
    fn mode_system_rejects_not_square() {
        let mech = crank(false);
        assert_eq!(
            ModeSystem::new(&mech).unwrap_err(),
            SolveError::NotSquare {
                equations: 7,
                unknowns: 8
            }
        );
    }

    #[test]
    fn mode_system_rejects_invalid_model() {
        let mut mech = crank(true);
        if let ElementKind::Beam { a, .. } = &mut mech.elements[0].kind {
            *a = CoordId(999);
        }
        assert_eq!(
            ModeSystem::new(&mech).unwrap_err(),
            SolveError::InvalidModel(ModelError::InvalidCoordId { element: 1 })
        );
    }
}
