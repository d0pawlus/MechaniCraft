//! Drawable geometry of a mechanism: nodes, supports and links.

use mc_core::{CoordId, CoordLabel, ElementKind, Mechanism};

use crate::document::Document;

/// A node with its coordinates and supports.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SceneNode {
    pub nr: u32,
    pub x: CoordId,
    pub y: CoordId,
    pub fixed_x: bool,
    pub fixed_y: bool,
}

/// How a link is drawn.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LinkKind {
    Beam,
    Slider,
}

/// A two-node element (beam or slider) between nodes `n1` and `n2`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SceneLink {
    pub element_nr: u32,
    pub kind: LinkKind,
    pub n1: u32,
    pub n2: u32,
    p: [CoordId; 2],
    q: [CoordId; 2],
}

/// A link placed in one state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinkSegment {
    pub element_nr: u32,
    pub kind: LinkKind,
    pub p: [f64; 2],
    pub q: [f64; 2],
}

/// Axis-aligned rectangle in model coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bounds {
    pub min: [f64; 2],
    pub max: [f64; 2],
}

impl Bounds {
    fn of_point(p: [f64; 2]) -> Self {
        Self { min: p, max: p }
    }

    fn including(self, p: [f64; 2]) -> Self {
        Self {
            min: [self.min[0].min(p[0]), self.min[1].min(p[1])],
            max: [self.max[0].max(p[0]), self.max[1].max(p[1])],
        }
    }
}

/// Nodes and links of a mechanism, ordered by number.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Scene {
    pub nodes: Vec<SceneNode>,
    pub links: Vec<SceneLink>,
}

impl Scene {
    /// Collects nodes (sorted by number) and beam/slider links (in element
    /// order). Hinges and motors have no geometry of their own.
    pub fn from_mechanism(mech: &Mechanism) -> Self {
        let mut nodes: Vec<SceneNode> = mech
            .coords
            .iter()
            .filter_map(|c| match c.label {
                CoordLabel::NodeX(nr) => {
                    let (x, y) = mech.node(nr)?;
                    Some(SceneNode {
                        nr,
                        x,
                        y,
                        fixed_x: mech.coords[x.0].fixed,
                        fixed_y: mech.coords[y.0].fixed,
                    })
                }
                CoordLabel::NodeY(_) | CoordLabel::Angle(_) => None,
            })
            .collect();
        nodes.sort_by_key(|n| n.nr);

        let node_of = |p: [CoordId; 2]| nodes.iter().find(|n| n.x == p[0]).map(|n| n.nr);
        let links = mech
            .elements
            .iter()
            .filter_map(|e| {
                let (kind, p, q) = match &e.kind {
                    ElementKind::Beam { p, q, .. } => (LinkKind::Beam, *p, *q),
                    ElementKind::Slider { p, q, .. } => (LinkKind::Slider, *p, *q),
                    ElementKind::Hinge { .. } | ElementKind::RotMotor { .. } => return None,
                };
                Some(SceneLink {
                    element_nr: e.nr,
                    kind,
                    n1: node_of(p)?,
                    n2: node_of(q)?,
                    p,
                    q,
                })
            })
            .collect();
        Self { nodes, links }
    }
}

fn point(q: &[f64], ids: [CoordId; 2]) -> Option<[f64; 2]> {
    Some([*q.get(ids[0].0)?, *q.get(ids[1].0)?])
}

impl Document {
    /// Position of node `nr` in `frame`; `None` for an unknown node or frame.
    pub fn node_position(&self, frame: usize, nr: u32) -> Option<[f64; 2]> {
        let state = self.trajectory.states.get(frame)?;
        let node = self.scene.nodes.iter().find(|n| n.nr == nr)?;
        point(&state.q, [node.x, node.y])
    }

    /// Links placed in `frame`; empty for an unknown frame.
    pub fn link_segments(&self, frame: usize) -> Vec<LinkSegment> {
        let Some(state) = self.trajectory.states.get(frame) else {
            return Vec::new();
        };
        self.scene
            .links
            .iter()
            .filter_map(|l| {
                Some(LinkSegment {
                    element_nr: l.element_nr,
                    kind: l.kind,
                    p: point(&state.q, l.p)?,
                    q: point(&state.q, l.q)?,
                })
            })
            .collect()
    }

    /// Rectangle containing every node in every frame; `None` without nodes
    /// or states.
    pub fn bounds(&self) -> Option<Bounds> {
        self.trajectory
            .states
            .iter()
            .flat_map(|s| {
                self.scene
                    .nodes
                    .iter()
                    .filter_map(move |n| point(&s.q, [n.x, n.y]))
            })
            .filter(|p| p[0].is_finite() && p[1].is_finite())
            .fold(None, |acc: Option<Bounds>, p| {
                Some(acc.map_or(Bounds::of_point(p), |b| b.including(p)))
            })
    }

    /// Path of node `nr` over all frames; empty for an unknown node.
    pub fn node_path(&self, nr: u32) -> Vec<[f64; 2]> {
        (0..self.frame_count())
            .map_while(|frame| self.node_position(frame, nr))
            .collect()
    }
}
