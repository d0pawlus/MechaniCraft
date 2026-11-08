//! Load model: masses, inertias, gravity and tabulated nodal forces.
//!
//! [`Loads`] is plain data. [`Loads::validate`] checks it against a
//! [`Mechanism`] and reports the first problem as a [`LoadError`].

use std::collections::HashSet;

use mc_core::{ElementKind, Mechanism};
use thiserror::Error;

/// Point mass attached to a node.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NodeMass {
    pub node: u32,
    pub mass: f64,
}

/// Mass and rotational inertia of a beam or slider.
///
/// `cog` is the fraction of the element length measured from node 1 (`XL`),
/// so it must lie in `[0, 1]`. `inertia` is the moment of inertia about the
/// centre of gravity.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ElementInertia {
    pub element: u32,
    pub mass: f64,
    pub cog: f64,
    pub inertia: f64,
}

/// Uniform gravity field `value · (cos angle, sin angle)`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gravity {
    pub value: f64,
    pub angle: f64,
}

impl Gravity {
    /// Gravity acceleration vector `value · (cos angle, sin angle)`.
    pub fn vector(&self) -> [f64; 2] {
        [self.value * self.angle.cos(), self.value * self.angle.sin()]
    }
}

/// Argument of a force table: time, or the `X` or `Y` coordinate of a node.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ForceArg {
    Time,
    NodeX(u32),
    NodeY(u32),
}

/// Tabulated force magnitude and direction.
///
/// Each row is `[argument, value, angle]`, with the angle in absolute radians.
/// Arguments increase strictly from row to row.
#[derive(Debug, Clone, PartialEq)]
pub struct ForceTable {
    rows: Vec<[f64; 3]>,
}

impl ForceTable {
    /// Builds a table from `rows`.
    ///
    /// Returns [`LoadError::EmptyForceTable`] for an empty table and
    /// [`LoadError::BadForceTableRow`] for the first row that has a
    /// non-finite entry or whose argument does not exceed the previous one.
    pub fn new(rows: Vec<[f64; 3]>) -> Result<Self, LoadError> {
        if rows.is_empty() {
            return Err(LoadError::EmptyForceTable);
        }
        for (i, row) in rows.iter().enumerate() {
            if !row.iter().all(|v| v.is_finite()) {
                return Err(LoadError::BadForceTableRow { row: i });
            }
            if i > 0 && row[0] <= rows[i - 1][0] {
                return Err(LoadError::BadForceTableRow { row: i });
            }
        }
        Ok(Self { rows })
    }

    /// Table rows as `[argument, value, angle]`.
    pub fn rows(&self) -> &[[f64; 3]] {
        &self.rows
    }

    /// Returns `(value, angle)` at argument `arg`.
    ///
    /// Values are interpolated linearly between neighbouring rows. Outside
    /// the table range the nearest end row is used, and a one-row table is a
    /// constant. A NaN argument returns the first row.
    pub fn eval(&self, arg: f64) -> (f64, f64) {
        // `new` guarantees at least one row, so the indexing below is safe.
        let first = self.rows[0];
        let last = self.rows[self.rows.len() - 1];
        if arg.is_nan() || arg <= first[0] {
            return (first[1], first[2]);
        }
        if arg >= last[0] {
            return (last[1], last[2]);
        }
        // Here first[0] < arg < last[0], so the partition point lies in 1..len.
        let hi = self.rows.partition_point(|r| r[0] <= arg);
        let lo = self.rows[hi - 1];
        let next = self.rows[hi];
        let t = (arg - lo[0]) / (next[0] - lo[0]);
        (lo[1] + t * (next[1] - lo[1]), lo[2] + t * (next[2] - lo[2]))
    }
}

/// Force applied at a node, with magnitude and direction taken from a table.
#[derive(Debug, Clone, PartialEq)]
pub struct NodalForce {
    pub node: u32,
    pub arg: ForceArg,
    pub table: ForceTable,
}

/// All loads of a mechanism.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Loads {
    pub node_masses: Vec<NodeMass>,
    pub element_inertias: Vec<ElementInertia>,
    pub gravity: Option<Gravity>,
    pub forces: Vec<NodalForce>,
}

impl Loads {
    /// Checks the loads against `mech`.
    ///
    /// Groups are checked in this order: node masses, element inertias,
    /// gravity, forces. Within a group the first failing entry is reported.
    pub fn validate(&self, mech: &Mechanism) -> Result<(), LoadError> {
        for m in &self.node_masses {
            if mech.node(m.node).is_none() {
                return Err(LoadError::UnknownNode(m.node));
            }
            finite_non_negative(m.mass, || format!("mass of node {}", m.node))?;
        }

        let mut seen = HashSet::new();
        for ei in &self.element_inertias {
            let element = mech
                .elements
                .iter()
                .find(|e| e.nr == ei.element)
                .ok_or(LoadError::UnknownElement(ei.element))?;
            let is_slider = match &element.kind {
                ElementKind::Beam { .. } => false,
                ElementKind::Slider { .. } => true,
                ElementKind::Hinge { .. } | ElementKind::RotMotor { .. } => {
                    return Err(LoadError::NotABeamOrSlider(ei.element));
                }
            };
            if !seen.insert(ei.element) {
                return Err(LoadError::DuplicateElementInertia(ei.element));
            }
            finite_non_negative(ei.mass, || format!("mass of element {}", ei.element))?;
            finite_non_negative(ei.inertia, || {
                format!("inertia of element {}", ei.element)
            })?;
            if !(0.0..=1.0).contains(&ei.cog) {
                return Err(LoadError::CogOutOfRange {
                    element: ei.element,
                    cog: ei.cog,
                });
            }
            // Sliders are only supported with the centre of gravity at node 1,
            // because the slider's own inertia model is not implemented.
            if is_slider && ei.cog != 0.0 {
                return Err(LoadError::SliderCog(ei.element));
            }
        }

        if let Some(g) = self.gravity {
            if !(g.value.is_finite() && g.angle.is_finite()) {
                return Err(LoadError::NonFiniteGravity);
            }
        }

        for f in &self.forces {
            if mech.node(f.node).is_none() {
                return Err(LoadError::UnknownNode(f.node));
            }
            match f.arg {
                ForceArg::Time => {}
                ForceArg::NodeX(n) | ForceArg::NodeY(n) => {
                    if mech.node(n).is_none() {
                        return Err(LoadError::UnknownNode(n));
                    }
                }
            }
        }
        Ok(())
    }

    /// True when no load is defined.
    pub fn is_empty(&self) -> bool {
        self.node_masses.is_empty()
            && self.element_inertias.is_empty()
            && self.gravity.is_none()
            && self.forces.is_empty()
    }
}

/// Errors reported by [`Loads::validate`] and [`ForceTable::new`].
#[derive(Debug, Clone, PartialEq, Error)]
pub enum LoadError {
    #[error("load refers to unknown node {0}")]
    UnknownNode(u32),
    #[error("inertia refers to unknown element {0}")]
    UnknownElement(u32),
    #[error("element {0} cannot carry mass: only beams and sliders can")]
    NotABeamOrSlider(u32),
    #[error("duplicate inertia for element {0}")]
    DuplicateElementInertia(u32),
    #[error("{what} must be finite and non-negative, got {value}")]
    InvalidValue { what: String, value: f64 },
    #[error("centre of gravity of element {element} must lie in [0, 1], got {cog}")]
    CogOutOfRange { element: u32, cog: f64 },
    #[error("slider {0}: a centre of gravity away from node 1 is not supported")]
    SliderCog(u32),
    #[error("gravity must be finite")]
    NonFiniteGravity,
    #[error("force table is empty")]
    EmptyForceTable,
    #[error("force table row {row} is not finite or its argument does not increase")]
    BadForceTableRow { row: usize },
}

/// Accepts `value` only when it is finite and non-negative. `what` is called
/// only to build the error message.
fn finite_non_negative(value: f64, what: impl FnOnce() -> String) -> Result<(), LoadError> {
    if value.is_finite() && value >= 0.0 {
        Ok(())
    } else {
        Err(LoadError::InvalidValue {
            what: what(),
            value,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use mc_core::MechanismBuilder;
    use mc_motion::{InputLaw, Segment};
    use std::f64::consts::FRAC_PI_2;

    fn drive_law() -> InputLaw {
        InputLaw {
            segments: vec![Segment::Linear {
                duration: 1.0,
                steps: 10,
                displacement: 1.0,
            }],
        }
    }

    /// Slider-crank: fixed pivot node 1, crank beam 1, rod beam 2, slider node 3.
    fn slider_crank() -> Mechanism {
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
            .input(3, drive_law());
        b.build().unwrap()
    }

    /// Mechanism with one slider (nr 5) between nodes 1 and 2.
    fn with_slider() -> Mechanism {
        let mut b = MechanismBuilder::new();
        b.node(1, 0.0, 0.0, true, true)
            .node(2, 0.3, 0.0, false, true)
            .angle(1001, 0.0, false)
            .angle(1002, 0.0, false)
            .slider(5, 1, 2, 1001, 1002);
        b.build().unwrap()
    }

    fn valid_loads() -> Loads {
        Loads {
            node_masses: vec![NodeMass {
                node: 3,
                mass: 25.0,
            }],
            element_inertias: vec![
                ElementInertia {
                    element: 1,
                    mass: 1.0,
                    cog: 0.5,
                    inertia: 0.01,
                },
                ElementInertia {
                    element: 2,
                    mass: 2.0,
                    cog: 0.3,
                    inertia: 0.05,
                },
            ],
            gravity: Some(Gravity {
                value: 9.81,
                angle: -FRAC_PI_2,
            }),
            forces: vec![NodalForce {
                node: 3,
                arg: ForceArg::NodeY(2),
                table: ForceTable::new(vec![[0.0, 1.0, 0.0]]).unwrap(),
            }],
        }
    }

    #[test]
    fn force_table_interpolates_and_clamps() {
        let t = ForceTable::new(vec![
            [0.0, 10.0, 0.0],
            [1.0, 30.0, 1.0],
            [3.0, 70.0, 2.0],
        ])
        .unwrap();
        let check = |got: (f64, f64), want: (f64, f64)| {
            assert_abs_diff_eq!(got.0, want.0, epsilon = 1e-12);
            assert_abs_diff_eq!(got.1, want.1, epsilon = 1e-12);
        };
        check(t.eval(0.5), (20.0, 0.5));
        check(t.eval(2.0), (50.0, 1.5));
        check(t.eval(-1.0), (10.0, 0.0));
        check(t.eval(5.0), (70.0, 2.0));
        check(t.eval(1.0), (30.0, 1.0));
    }

    #[test]
    fn force_table_single_row_is_constant() {
        let t = ForceTable::new(vec![[2.0, 5.0, -0.5]]).unwrap();
        assert_eq!(t.eval(-10.0), (5.0, -0.5));
        assert_eq!(t.eval(10.0), (5.0, -0.5));
    }

    #[test]
    fn force_table_rejects_bad_rows() {
        assert_eq!(ForceTable::new(vec![]), Err(LoadError::EmptyForceTable));
        assert_eq!(
            ForceTable::new(vec![[1.0, 0.0, 0.0], [1.0, 1.0, 0.0]]),
            Err(LoadError::BadForceTableRow { row: 1 })
        );
        assert_eq!(
            ForceTable::new(vec![[0.0, f64::NAN, 0.0]]),
            Err(LoadError::BadForceTableRow { row: 0 })
        );
    }

    #[test]
    fn gravity_vector_points_down() {
        let g = Gravity {
            value: 9.81,
            angle: -FRAC_PI_2,
        };
        let v = g.vector();
        assert_abs_diff_eq!(v[0], 0.0, epsilon = 1e-12);
        assert_abs_diff_eq!(v[1], -9.81, epsilon = 1e-12);
    }

    #[test]
    fn validate_accepts_slider_crank_loads() {
        assert_eq!(valid_loads().validate(&slider_crank()), Ok(()));
        assert!(!valid_loads().is_empty());
        assert!(Loads::default().is_empty());
    }

    #[test]
    fn validate_rejects_bad_loads() {
        let mech = slider_crank();

        let mut l = valid_loads();
        l.node_masses[0].node = 99;
        assert_eq!(l.validate(&mech), Err(LoadError::UnknownNode(99)));

        let mut l = valid_loads();
        l.element_inertias[0].element = 42;
        assert_eq!(l.validate(&mech), Err(LoadError::UnknownElement(42)));

        let mut l = valid_loads();
        l.element_inertias.push(ElementInertia {
            element: 3,
            mass: 1.0,
            cog: 0.5,
            inertia: 0.0,
        });
        assert_eq!(l.validate(&mech), Err(LoadError::NotABeamOrSlider(3)));

        let mut l = valid_loads();
        l.element_inertias.push(ElementInertia {
            element: 1,
            mass: 1.0,
            cog: 0.5,
            inertia: 0.0,
        });
        assert_eq!(
            l.validate(&mech),
            Err(LoadError::DuplicateElementInertia(1))
        );

        let mut l = valid_loads();
        l.node_masses[0].mass = -1.0;
        assert!(matches!(
            l.validate(&mech),
            Err(LoadError::InvalidValue { .. })
        ));

        let mut l = valid_loads();
        l.element_inertias[0].cog = 1.5;
        assert_eq!(
            l.validate(&mech),
            Err(LoadError::CogOutOfRange {
                element: 1,
                cog: 1.5
            })
        );

        let mut l = valid_loads();
        l.gravity = Some(Gravity {
            value: f64::INFINITY,
            angle: 0.0,
        });
        assert_eq!(l.validate(&mech), Err(LoadError::NonFiniteGravity));

        let mut l = valid_loads();
        l.forces[0].arg = ForceArg::NodeX(77);
        assert_eq!(l.validate(&mech), Err(LoadError::UnknownNode(77)));
    }

    #[test]
    fn validate_rejects_slider_cog() {
        let mech = with_slider();
        let slider = ElementInertia {
            element: 5,
            mass: 1.0,
            cog: 0.5,
            inertia: 0.0,
        };
        let loads = Loads {
            element_inertias: vec![slider],
            ..Loads::default()
        };
        assert_eq!(loads.validate(&mech), Err(LoadError::SliderCog(5)));

        let loads = Loads {
            element_inertias: vec![ElementInertia { cog: 0.0, ..slider }],
            ..Loads::default()
        };
        assert_eq!(loads.validate(&mech), Ok(()));
    }
}
