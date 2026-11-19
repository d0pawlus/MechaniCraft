//! Time series that can be plotted: coordinates and their derivatives,
//! driving forces and power.

use mc_core::{CoordId, CoordLabel};

use crate::document::Document;

/// Which time derivative of a coordinate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Derivative {
    Value,
    Rate,
    Accel,
}

impl Derivative {
    const ALL: [Self; 3] = [Self::Value, Self::Rate, Self::Accel];

    fn suffix(self) -> &'static str {
        match self {
            Self::Value => "",
            Self::Rate => "'",
            Self::Accel => "''",
        }
    }
}

/// One plottable series.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Channel {
    /// A free coordinate (node x/y or angle) or one of its derivatives.
    Coord { id: CoordId, derivative: Derivative },
    /// Driving force or torque of input `i`.
    Driving(usize),
    /// Driving power of input `i`.
    Power(usize),
}

impl Document {
    /// Channels in display order: free coordinates (each with value, rate and
    /// acceleration), then driving force and power of every input when forces
    /// are available. Fixed coordinates are constant and left out.
    pub fn channels(&self) -> Vec<Channel> {
        let coords = self
            .project
            .mechanism
            .coords
            .iter()
            .enumerate()
            .filter(|(_, c)| !c.fixed)
            .flat_map(|(i, _)| {
                Derivative::ALL.map(|derivative| Channel::Coord {
                    id: CoordId(i),
                    derivative,
                })
            });
        let n_inputs = match &self.forces {
            Some(_) => self.project.mechanism.inputs.len(),
            None => 0,
        };
        let forces = (0..n_inputs).flat_map(|i| [Channel::Driving(i), Channel::Power(i)]);
        coords.chain(forces).collect()
    }

    /// Channel shown when a document opens: x of the highest-numbered node
    /// that moves in x (typically the output slider), else the first channel.
    pub fn default_channel(&self) -> Option<Channel> {
        self.scene
            .nodes
            .iter()
            .rev()
            .find(|n| !n.fixed_x)
            .and_then(|n| self.node_channel(n.nr, 'x', Derivative::Value))
            .or_else(|| self.channels().first().copied())
    }

    /// Node whose path is traced when a document opens: the first node free
    /// in both x and y.
    pub fn default_path_node(&self) -> Option<u32> {
        self.scene
            .nodes
            .iter()
            .find(|n| !n.fixed_x && !n.fixed_y)
            .map(|n| n.nr)
    }

    /// Channel of node `nr`, axis `'x'` or `'y'`.
    pub fn node_channel(&self, nr: u32, axis: char, derivative: Derivative) -> Option<Channel> {
        let (x, y) = self.project.mechanism.node(nr)?;
        let id = match axis {
            'x' => x,
            'y' => y,
            _ => return None,
        };
        Some(Channel::Coord { id, derivative })
    }

    /// Short label: `x3`, `y2'` (velocity), `A1001''` (acceleration),
    /// `DRV3` and `PWR3` (driving force and power of the input on element 3,
    /// as in the CLI output).
    pub fn label(&self, channel: Channel) -> String {
        let mech = &self.project.mechanism;
        match channel {
            Channel::Coord { id, derivative } => {
                let name = match mech.coords.get(id.0).map(|c| c.label) {
                    Some(CoordLabel::NodeX(nr)) => format!("x{nr}"),
                    Some(CoordLabel::NodeY(nr)) => format!("y{nr}"),
                    Some(CoordLabel::Angle(nr)) => format!("A{nr}"),
                    None => format!("?{}", id.0),
                };
                format!("{name}{}", derivative.suffix())
            }
            Channel::Driving(i) => format!("DRV{}", input_nr(self, i)),
            Channel::Power(i) => format!("PWR{}", input_nr(self, i)),
        }
    }

    /// `(t, value)` samples of `channel`; empty when the data is missing.
    pub fn samples(&self, channel: Channel) -> Vec<[f64; 2]> {
        match channel {
            Channel::Coord { id, derivative } => self
                .trajectory
                .states
                .iter()
                .map_while(|s| {
                    let v = match derivative {
                        Derivative::Value => &s.q,
                        Derivative::Rate => &s.qd,
                        Derivative::Accel => &s.qdd,
                    };
                    Some([s.t, *v.get(id.0)?])
                })
                .collect(),
            Channel::Driving(i) => self.force_samples(|f| f.driving.get(i).copied()),
            Channel::Power(i) => self.force_samples(|f| f.power.get(i).copied()),
        }
    }

    fn force_samples(&self, pick: impl Fn(&mc_kineto::ForceState) -> Option<f64>) -> Vec<[f64; 2]> {
        let Some(forces) = &self.forces else {
            return Vec::new();
        };
        forces
            .states
            .iter()
            .map_while(|f| Some([f.t, pick(f)?]))
            .collect()
    }
}

fn input_nr(doc: &Document, i: usize) -> String {
    doc.project
        .mechanism
        .inputs
        .get(i)
        .map_or_else(|| format!("?{i}"), |inp| inp.element_nr.to_string())
}
