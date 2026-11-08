//! Mechanism model and builder.
//!
//! A [`Mechanism`] is a flat list of coordinates (node `x`/`y` and element
//! nodal angles), the elements that constrain them, and the inputs that drive
//! some of the elements. [`MechanismBuilder`] records numbered declarations
//! and [`MechanismBuilder::build`] validates them and resolves all numbers to
//! [`CoordId`]s.

use std::collections::{HashMap, HashSet};

use mc_motion::InputLaw;
use thiserror::Error;

/// Length below which a beam or slider is considered degenerate.
const ZERO_LENGTH_TOL: f64 = 1e-12;

/// Index of a coordinate in [`Mechanism::coords`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CoordId(pub usize);

/// What a coordinate represents.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CoordLabel {
    NodeX(u32),
    NodeY(u32),
    Angle(u32),
}

/// One coordinate of the mechanism with its initial value.
#[derive(Debug, Clone, PartialEq)]
pub struct Coord {
    pub label: CoordLabel,
    pub value0: f64,
    pub fixed: bool,
}

/// Element type together with the coordinates it refers to.
#[derive(Debug, Clone, PartialEq)]
pub enum ElementKind {
    Beam {
        p: [CoordId; 2],
        q: [CoordId; 2],
        a: CoordId,
        b: CoordId,
    },
    Slider {
        p: [CoordId; 2],
        q: [CoordId; 2],
        a: CoordId,
        b: CoordId,
    },
    Hinge {
        a: CoordId,
        b: CoordId,
        fixed: bool,
    },
    RotMotor {
        a: CoordId,
        b: CoordId,
        fixed: bool,
    },
}

/// A numbered element of the mechanism.
#[derive(Debug, Clone, PartialEq)]
pub struct Element {
    pub nr: u32,
    pub kind: ElementKind,
}

/// A motion law applied to an element (deformation mode `KIND:1`).
#[derive(Debug, Clone, PartialEq)]
pub struct Input {
    pub element_nr: u32,
    pub law: InputLaw,
}

/// A validated planar mechanism.
#[derive(Debug, Clone, PartialEq)]
pub struct Mechanism {
    pub coords: Vec<Coord>,
    pub elements: Vec<Element>,
    pub inputs: Vec<Input>,
}

impl Mechanism {
    /// Returns the `(x, y)` coordinate ids of node `nr`.
    pub fn node(&self, nr: u32) -> Option<(CoordId, CoordId)> {
        let x = self.find(CoordLabel::NodeX(nr))?;
        let y = self.find(CoordLabel::NodeY(nr))?;
        Some((x, y))
    }

    /// Returns the coordinate id of angle `nr`.
    pub fn angle(&self, nr: u32) -> Option<CoordId> {
        self.find(CoordLabel::Angle(nr))
    }

    fn find(&self, label: CoordLabel) -> Option<CoordId> {
        self.coords
            .iter()
            .position(|c| c.label == label)
            .map(CoordId)
    }
}

/// Errors reported by [`MechanismBuilder::build`].
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum ModelError {
    #[error("duplicate node number {0}")]
    DuplicateNode(u32),
    #[error("duplicate angle number {0}")]
    DuplicateAngle(u32),
    #[error("duplicate element number {0}")]
    DuplicateElement(u32),
    #[error("element {element} refers to unknown node {node}")]
    UnknownNode { element: u32, node: u32 },
    #[error("element {element} refers to unknown angle {angle}")]
    UnknownAngle { element: u32, angle: u32 },
    #[error("input refers to unknown element {0}")]
    UnknownInputElement(u32),
    #[error("beam or slider {0} has zero length in its initial configuration")]
    ZeroLengthBeam(u32),
}

#[derive(Debug, Clone, Copy)]
struct NodeSpec {
    nr: u32,
    x: f64,
    y: f64,
    fix_x: bool,
    fix_y: bool,
}

#[derive(Debug, Clone, Copy)]
struct AngleSpec {
    nr: u32,
    a0: f64,
    fixed: bool,
}

#[derive(Debug, Clone, Copy)]
enum ElementSpec {
    Beam {
        nr: u32,
        nodes: [u32; 2],
        angles: [u32; 2],
    },
    Slider {
        nr: u32,
        nodes: [u32; 2],
        angles: [u32; 2],
    },
    Hinge {
        nr: u32,
        angles: [u32; 2],
        fixed: bool,
    },
    RotMotor {
        nr: u32,
        angles: [u32; 2],
        fixed: bool,
    },
}

impl ElementSpec {
    fn nr(&self) -> u32 {
        match *self {
            ElementSpec::Beam { nr, .. }
            | ElementSpec::Slider { nr, .. }
            | ElementSpec::Hinge { nr, .. }
            | ElementSpec::RotMotor { nr, .. } => nr,
        }
    }

    fn node_refs(&self) -> &[u32] {
        match self {
            ElementSpec::Beam { nodes, .. } | ElementSpec::Slider { nodes, .. } => &nodes[..],
            ElementSpec::Hinge { .. } | ElementSpec::RotMotor { .. } => &[],
        }
    }

    fn angle_refs(&self) -> &[u32] {
        match self {
            ElementSpec::Beam { angles, .. }
            | ElementSpec::Slider { angles, .. }
            | ElementSpec::Hinge { angles, .. }
            | ElementSpec::RotMotor { angles, .. } => &angles[..],
        }
    }
}

/// Collects numbered mechanism declarations and builds a [`Mechanism`].
///
/// Methods only record calls. All checks happen in [`MechanismBuilder::build`].
#[derive(Debug, Clone, Default)]
pub struct MechanismBuilder {
    nodes: Vec<NodeSpec>,
    angles: Vec<AngleSpec>,
    elements: Vec<ElementSpec>,
    inputs: Vec<Input>,
}

impl MechanismBuilder {
    /// Creates an empty builder.
    pub fn new() -> Self {
        Self::default()
    }

    /// Declares node `nr` at `(x, y)`. Each coordinate can be fixed separately.
    pub fn node(&mut self, nr: u32, x: f64, y: f64, fix_x: bool, fix_y: bool) -> &mut Self {
        self.nodes.push(NodeSpec {
            nr,
            x,
            y,
            fix_x,
            fix_y,
        });
        self
    }

    /// Declares angle `nr` with initial value `a0`.
    pub fn angle(&mut self, nr: u32, a0: f64, fixed: bool) -> &mut Self {
        self.angles.push(AngleSpec { nr, a0, fixed });
        self
    }

    /// Declares a beam `nr` between nodes `n1`, `n2` with angles `a1`, `a2`.
    pub fn beam(&mut self, nr: u32, n1: u32, n2: u32, a1: u32, a2: u32) -> &mut Self {
        self.elements.push(ElementSpec::Beam {
            nr,
            nodes: [n1, n2],
            angles: [a1, a2],
        });
        self
    }

    /// Declares a slider `nr` between nodes `n1`, `n2` with angles `a1`, `a2`.
    pub fn slider(&mut self, nr: u32, n1: u32, n2: u32, a1: u32, a2: u32) -> &mut Self {
        self.elements.push(ElementSpec::Slider {
            nr,
            nodes: [n1, n2],
            angles: [a1, a2],
        });
        self
    }

    /// Declares a hinge `nr` between angles `a1`, `a2`.
    pub fn hinge(&mut self, nr: u32, a1: u32, a2: u32, fixed: bool) -> &mut Self {
        self.elements.push(ElementSpec::Hinge {
            nr,
            angles: [a1, a2],
            fixed,
        });
        self
    }

    /// Declares a rotational motor `nr` between angles `a1`, `a2`.
    pub fn rot_motor(&mut self, nr: u32, a1: u32, a2: u32, fixed: bool) -> &mut Self {
        self.elements.push(ElementSpec::RotMotor {
            nr,
            angles: [a1, a2],
            fixed,
        });
        self
    }

    /// Applies motion law `law` to element `element_nr`.
    pub fn input(&mut self, element_nr: u32, law: InputLaw) -> &mut Self {
        self.inputs.push(Input { element_nr, law });
        self
    }

    /// Validates the declarations and resolves them to a [`Mechanism`].
    ///
    /// Checks run in this order: duplicates, references, zero-length
    /// beams and sliders, inputs.
    pub fn build(&self) -> Result<Mechanism, ModelError> {
        self.check_duplicates()?;
        self.check_references()?;
        self.check_zero_length()?;
        self.check_inputs()?;
        self.resolve()
    }

    fn node_spec(&self, nr: u32) -> Option<&NodeSpec> {
        self.nodes.iter().find(|n| n.nr == nr)
    }

    fn angle_spec(&self, nr: u32) -> Option<&AngleSpec> {
        self.angles.iter().find(|a| a.nr == nr)
    }

    fn check_duplicates(&self) -> Result<(), ModelError> {
        let mut seen = HashSet::new();
        for n in &self.nodes {
            if !seen.insert(n.nr) {
                return Err(ModelError::DuplicateNode(n.nr));
            }
        }
        seen.clear();
        for a in &self.angles {
            if !seen.insert(a.nr) {
                return Err(ModelError::DuplicateAngle(a.nr));
            }
        }
        seen.clear();
        for e in &self.elements {
            if !seen.insert(e.nr()) {
                return Err(ModelError::DuplicateElement(e.nr()));
            }
        }
        Ok(())
    }

    fn check_references(&self) -> Result<(), ModelError> {
        for e in &self.elements {
            for &node in e.node_refs() {
                if self.node_spec(node).is_none() {
                    return Err(ModelError::UnknownNode {
                        element: e.nr(),
                        node,
                    });
                }
            }
            for &angle in e.angle_refs() {
                if self.angle_spec(angle).is_none() {
                    return Err(ModelError::UnknownAngle {
                        element: e.nr(),
                        angle,
                    });
                }
            }
        }
        Ok(())
    }

    fn check_zero_length(&self) -> Result<(), ModelError> {
        for e in &self.elements {
            let nodes = match e {
                ElementSpec::Beam { nodes, .. } | ElementSpec::Slider { nodes, .. } => nodes,
                ElementSpec::Hinge { .. } | ElementSpec::RotMotor { .. } => continue,
            };
            // References were validated above, so both lookups succeed.
            if let (Some(p), Some(q)) = (self.node_spec(nodes[0]), self.node_spec(nodes[1])) {
                let length = (q.x - p.x).hypot(q.y - p.y);
                if length < ZERO_LENGTH_TOL {
                    return Err(ModelError::ZeroLengthBeam(e.nr()));
                }
            }
        }
        Ok(())
    }

    fn check_inputs(&self) -> Result<(), ModelError> {
        for input in &self.inputs {
            self.elements
                .iter()
                .find(|e| e.nr() == input.element_nr)
                .ok_or(ModelError::UnknownInputElement(input.element_nr))?;
        }
        Ok(())
    }

    /// Assigns coordinate ids in call order and resolves element references.
    /// Runs only after validation; the lookup errors are a safety net, not a
    /// reachable path.
    fn resolve(&self) -> Result<Mechanism, ModelError> {
        let mut coords = Vec::with_capacity(2 * self.nodes.len() + self.angles.len());
        let mut node_ids: HashMap<u32, (CoordId, CoordId)> = HashMap::new();
        let mut angle_ids: HashMap<u32, CoordId> = HashMap::new();

        for n in &self.nodes {
            let x_id = CoordId(coords.len());
            coords.push(Coord {
                label: CoordLabel::NodeX(n.nr),
                value0: n.x,
                fixed: n.fix_x,
            });
            let y_id = CoordId(coords.len());
            coords.push(Coord {
                label: CoordLabel::NodeY(n.nr),
                value0: n.y,
                fixed: n.fix_y,
            });
            node_ids.insert(n.nr, (x_id, y_id));
        }
        for a in &self.angles {
            let id = CoordId(coords.len());
            coords.push(Coord {
                label: CoordLabel::Angle(a.nr),
                value0: a.a0,
                fixed: a.fixed,
            });
            angle_ids.insert(a.nr, id);
        }

        let node = |element: u32, nr: u32| {
            node_ids
                .get(&nr)
                .copied()
                .ok_or(ModelError::UnknownNode { element, node: nr })
        };
        let angle = |element: u32, nr: u32| {
            angle_ids
                .get(&nr)
                .copied()
                .ok_or(ModelError::UnknownAngle { element, angle: nr })
        };

        let mut elements = Vec::with_capacity(self.elements.len());
        for e in &self.elements {
            let kind = match *e {
                ElementSpec::Beam { nr, nodes, angles } => {
                    let p = node(nr, nodes[0])?;
                    let q = node(nr, nodes[1])?;
                    ElementKind::Beam {
                        p: [p.0, p.1],
                        q: [q.0, q.1],
                        a: angle(nr, angles[0])?,
                        b: angle(nr, angles[1])?,
                    }
                }
                ElementSpec::Slider { nr, nodes, angles } => {
                    let p = node(nr, nodes[0])?;
                    let q = node(nr, nodes[1])?;
                    ElementKind::Slider {
                        p: [p.0, p.1],
                        q: [q.0, q.1],
                        a: angle(nr, angles[0])?,
                        b: angle(nr, angles[1])?,
                    }
                }
                ElementSpec::Hinge { nr, angles, fixed } => ElementKind::Hinge {
                    a: angle(nr, angles[0])?,
                    b: angle(nr, angles[1])?,
                    fixed,
                },
                ElementSpec::RotMotor { nr, angles, fixed } => ElementKind::RotMotor {
                    a: angle(nr, angles[0])?,
                    b: angle(nr, angles[1])?,
                    fixed,
                },
            };
            elements.push(Element { nr: e.nr(), kind });
        }

        Ok(Mechanism {
            coords,
            elements,
            inputs: self.inputs.clone(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mc_motion::Segment;
    use std::f64::consts::PI;

    fn drive_law() -> InputLaw {
        InputLaw {
            segments: vec![Segment::Linear {
                duration: 1.0,
                steps: 10,
                displacement: 2.0 * PI,
            }],
        }
    }

    /// Slider-crank: fixed crank pivot node 1, crank beam 1, rod beam 2, slider node 3.
    fn slider_crank_builder() -> MechanismBuilder {
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
        b
    }

    #[test]
    fn builds_slider_crank() {
        let m = slider_crank_builder().build().unwrap();
        assert_eq!(m.coords.len(), 11);
        assert_eq!(m.coords.iter().filter(|c| !c.fixed).count(), 7);
    }

    #[test]
    fn rejects_unknown_node() {
        let mut b = MechanismBuilder::new();
        b.node(1, 0.0, 0.0, true, true)
            .node(2, 0.1, 0.0, false, false)
            .angle(1001, 0.0, false)
            .angle(1002, 0.0, false)
            .beam(1, 1, 9, 1001, 1002);
        assert_eq!(
            b.build().unwrap_err(),
            ModelError::UnknownNode { element: 1, node: 9 }
        );
    }

    #[test]
    fn rejects_zero_length_beam() {
        let mut b = MechanismBuilder::new();
        b.node(1, 0.3, 0.2, true, true)
            .node(2, 0.3, 0.2, false, false)
            .angle(1001, 0.0, false)
            .angle(1002, 0.0, false)
            .beam(1, 1, 2, 1001, 1002);
        assert_eq!(b.build().unwrap_err(), ModelError::ZeroLengthBeam(1));
    }

    #[test]
    fn accepts_input_on_beam() {
        let mut b = MechanismBuilder::new();
        b.node(1, 0.0, 0.0, true, true)
            .node(2, 0.1, 0.0, false, false)
            .angle(1001, 0.0, false)
            .angle(1002, 0.0, false)
            .beam(1, 1, 2, 1001, 1002)
            .input(1, drive_law());
        assert!(b.build().is_ok());
    }

    #[test]
    fn node_and_angle_lookup() {
        let m = slider_crank_builder().build().unwrap();
        let (x3, y3) = m.node(3).unwrap();
        assert_eq!(m.coords[x3.0].label, CoordLabel::NodeX(3));
        assert_eq!(m.coords[y3.0].label, CoordLabel::NodeY(3));
        // Nodes 1..3 take indices 0..6, so angle 1005 (the fifth angle) is at 10.
        assert_eq!(m.angle(1005).map(|id| id.0), Some(10));
        assert_eq!(m.node(9), None);
        assert_eq!(m.angle(9999), None);
    }
}
