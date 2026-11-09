//! Conversion of a parsed SAM project into a [`Mechanism`].
//!
//! Phase 1 supports `BEAM`, `SLIDER`, `HINGE` and `ROTMOTOR` elements, inputs of
//! `KIND:1` and `CLINEAR` curve segments. Everything else is reported as
//! [`SamError::Unsupported`] with the line of the offending block.

use mc_core::{Mechanism, MechanismBuilder, ModelError};
use mc_kineto::{
    ElementInertia, ForceArg, ForceTable, Gravity, LoadError, Loads, NodalForce, NodeMass,
};
use mc_motion::{InputLaw, Segment};

use crate::tree::{SamError, SamNode, SamTree, parse_tree};

/// Errors produced while importing a SAM project.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum ImportError {
    /// The file could not be read or uses unsupported content.
    #[error(transparent)]
    Sam(#[from] SamError),
    /// The file was read but does not describe a valid mechanism.
    #[error(transparent)]
    Model(#[from] ModelError),
    /// The loads (masses, gravity, forces) are inconsistent with the mechanism.
    #[error(transparent)]
    Loads(#[from] LoadError),
}

/// A mechanism together with its loads, as read by [`import_project`].
#[derive(Debug, Clone, PartialEq)]
pub struct Project {
    pub mechanism: Mechanism,
    pub loads: Loads,
}

/// Imports the bytes of a `.SAM` ASCII file into a mechanism model.
pub fn import_sam(bytes: &[u8]) -> Result<Mechanism, ImportError> {
    let tree = parse_tree(bytes)?;
    let mut builder = MechanismBuilder::new();
    read_dofs(&tree, &mut builder)?;
    read_elements(&tree, &mut builder)?;
    read_inputs(&tree, &mut builder)?;
    Ok(builder.build()?)
}

/// Imports the mechanism (as [`import_sam`]) and its loads: node masses, beam and
/// slider inertias, gravity and tabulated nodal forces.
///
/// Loads that cannot be represented (external torques, forces outside a node,
/// force arguments other than a node coordinate, rotational inertia on hinges and
/// motors, a slider centre of gravity away from node 1) are reported as
/// [`SamError::Unsupported`] instead of being dropped.
pub fn import_project(bytes: &[u8]) -> Result<Project, ImportError> {
    let tree = parse_tree(bytes)?;
    let mut builder = MechanismBuilder::new();
    read_dofs(&tree, &mut builder)?;
    read_elements(&tree, &mut builder)?;
    read_inputs(&tree, &mut builder)?;
    let mechanism = builder.build()?;
    let loads = read_loads(&tree)?;
    loads.validate(&mechanism)?;
    Ok(Project { mechanism, loads })
}

/// Upper bound on `NSTEPS` of one curve segment, to keep memory use bounded.
const MAX_STEPS: u32 = 1_000_000;

fn required<'a>(tree: &'a SamTree, key: &str) -> Result<&'a SamNode, SamError> {
    tree.get(key).ok_or_else(|| SamError::Missing {
        key: key.to_owned(),
        line: 0,
    })
}

/// Reads a finite `f64` child; `NaN` and infinities are `BadNumber`.
fn finite(node: &SamNode, key: &str) -> Result<f64, SamError> {
    let value: f64 = node.num(key)?;
    if value.is_finite() {
        Ok(value)
    } else {
        Err(SamError::BadNumber {
            key: key.to_owned(),
            line: node.get(key).map_or(node.line, |n| n.line),
        })
    }
}

fn read_dofs(tree: &SamTree, builder: &mut MechanismBuilder) -> Result<(), SamError> {
    let doflist = required(tree, "DOFLIST")?;
    for child in &doflist.children {
        match child.key.as_str() {
            "NODE" => {
                let nr: u32 = child.num("NR")?;
                let x = finite(child, "X")?;
                let y = finite(child, "Y")?;
                let fix: u32 = child.num("FIXAXIS")?;
                builder.node(nr, x, y, fix & 1 != 0, fix & 2 != 0);
            }
            "ANGLE" => {
                let nr: u32 = child.num("NR")?;
                let a = finite(child, "A")?;
                let fix: u32 = child.num("FIXAXIS")?;
                builder.angle(nr, a, fix & 4 != 0);
            }
            _ => {}
        }
    }
    Ok(())
}

fn read_elements(tree: &SamTree, builder: &mut MechanismBuilder) -> Result<(), SamError> {
    let groups = required(tree, "ELEMGROUPS")?;
    for group in groups.all("ELEMGRP") {
        for list in group.all("ELEMLIST") {
            for el in &list.children {
                match el.key.as_str() {
                    "BEAM" | "SLIDER" => {
                        let nr: u32 = el.num("NR")?;
                        let (n1, n2): (u32, u32) = (el.num("N1")?, el.num("N2")?);
                        let (a1, a2): (u32, u32) = (el.num("A1")?, el.num("A2")?);
                        if el.key == "BEAM" {
                            builder.beam(nr, n1, n2, a1, a2);
                        } else {
                            builder.slider(nr, n1, n2, a1, a2);
                        }
                    }
                    "HINGE" | "ROTMOTOR" => {
                        let nr: u32 = el.num("NR")?;
                        let (a1, a2): (u32, u32) = (el.num("A1")?, el.num("A2")?);
                        let fixed = el.num::<u32>("FIXED")? != 0;
                        if el.key == "HINGE" {
                            builder.hinge(nr, a1, a2, fixed);
                        } else {
                            builder.rot_motor(nr, a1, a2, fixed);
                        }
                    }
                    other => {
                        return Err(SamError::Unsupported {
                            what: format!("element {other}"),
                            line: el.line,
                        });
                    }
                }
            }
        }
    }
    Ok(())
}

fn read_inputs(tree: &SamTree, builder: &mut MechanismBuilder) -> Result<(), SamError> {
    let Some(input) = tree.get("INPUT") else {
        return Ok(());
    };
    for block in input.all("MOTIONBLOCK") {
        let element: u32 = block.num("EDNR")?;
        let kind = block.get("KIND").ok_or_else(|| SamError::Missing {
            key: "KIND".to_owned(),
            line: block.line,
        })?;
        let kind_value: u32 = block.num("KIND")?;
        if kind_value != 1 {
            return Err(SamError::Unsupported {
                what: format!("input KIND {kind_value}"),
                line: kind.line,
            });
        }
        match block.get("AXIS") {
            None => {}
            Some(axis) => {
                let n: i64 = block.num("AXIS")?;
                if n != 0 {
                    return Err(SamError::Unsupported {
                        what: format!("input AXIS {n}"),
                        line: axis.line,
                    });
                }
            }
        }
        // Curve segments are siblings of CURVELIST (real files, below KIND), or
        // direct children of the block, or nested inside CURVELIST. All places
        // are searched and the result is kept in file order.
        let mut curves: Vec<&SamNode> = Vec::new();
        for parent in [block, kind] {
            for child in &parent.children {
                if child.key == "CURVELIST" {
                    curves.extend(child.children.iter());
                } else if child.key.starts_with('C') {
                    curves.push(child);
                }
            }
        }
        curves.sort_by_key(|curve| curve.line);
        let mut segments = Vec::with_capacity(curves.len());
        for curve in curves {
            if curve.key != "CLINEAR" {
                return Err(SamError::Unsupported {
                    what: format!("motion {}", curve.key),
                    line: curve.line,
                });
            }
            segments.push(read_linear(curve)?);
        }
        if segments.is_empty() {
            return Err(SamError::Missing {
                key: "CLINEAR".to_owned(),
                line: block.line,
            });
        }
        builder.input(element, InputLaw { segments });
    }
    Ok(())
}

fn read_linear(curve: &SamNode) -> Result<Segment, SamError> {
    let steps: u32 = curve.num("NSTEPS")?;
    let duration: f64 = curve.num("CTIME")?;
    let displacement: f64 = curve.num("MOTION")?;
    let bad = |key: &str| SamError::BadNumber {
        key: key.to_owned(),
        line: curve.get(key).map_or(curve.line, |n| n.line),
    };
    if !(1..=MAX_STEPS).contains(&steps) {
        return Err(bad("NSTEPS"));
    }
    if !duration.is_finite() || duration <= 0.0 {
        return Err(bad("CTIME"));
    }
    if !displacement.is_finite() {
        return Err(bad("MOTION"));
    }
    Ok(Segment::Linear {
        duration,
        steps,
        displacement,
    })
}

fn read_loads(tree: &SamTree) -> Result<Loads, ImportError> {
    let mut loads = Loads::default();
    let doflist = required(tree, "DOFLIST")?;
    for child in &doflist.children {
        match child.key.as_str() {
            "NODE" => {
                let nr: u32 = child.num("NR")?;
                if let Some(mass) = child.get("MASS") {
                    let value = finite(mass, "MASSVALUE")?;
                    if value != 0.0 {
                        loads.node_masses.push(NodeMass { node: nr, mass: value });
                    }
                }
                for block in child.all("FORCEBLOCK") {
                    if let Some(force) = read_force(block, nr)? {
                        loads.forces.push(force);
                    }
                }
            }
            "ANGLE" => {
                if let Some(mass) = child.get("MASS") {
                    return Err(SamError::Unsupported {
                        what: "MASS on ANGLE".to_owned(),
                        line: mass.line,
                    }
                    .into());
                }
            }
            _ => {}
        }
    }

    let groups = required(tree, "ELEMGROUPS")?;
    for group in groups.all("ELEMGRP") {
        for list in group.all("ELEMLIST") {
            for el in &list.children {
                match el.key.as_str() {
                    "BEAM" | "SLIDER" => {
                        let mass = optional_finite(el, "M")?;
                        let inertia = optional_finite(el, "J")?;
                        let cog = optional_finite(el, "XL")?;
                        if mass == 0.0 && inertia == 0.0 {
                            continue;
                        }
                        if el.key == "SLIDER" && mass != 0.0 && cog != 0.0 {
                            return Err(SamError::Unsupported {
                                what: "SLIDER centre of gravity XL".to_owned(),
                                line: el.get("XL").map_or(el.line, |n| n.line),
                            }
                            .into());
                        }
                        loads.element_inertias.push(ElementInertia {
                            element: el.num("NR")?,
                            mass,
                            cog,
                            inertia,
                        });
                    }
                    "HINGE" | "ROTMOTOR" => {
                        for key in ["J1", "J2"] {
                            if optional_finite(el, key)? != 0.0 {
                                return Err(SamError::Unsupported {
                                    what: format!("rotational inertia {key} on {}", el.key),
                                    line: el.get(key).map_or(el.line, |n| n.line),
                                }
                                .into());
                            }
                        }
                    }
                    _ => {}
                }
            }
        }
    }

    if let Some(gravity) = tree.get("GRAVITY") {
        let value = finite(gravity, "VALUE")?;
        if value != 0.0 {
            loads.gravity = Some(Gravity {
                value,
                angle: finite(gravity, "ANGLE")?,
            });
        }
    }

    for node in &tree.root {
        reject_stray_forceblocks(node, false)?;
    }
    Ok(loads)
}

/// Finite number of an optional child; an absent key reads as zero.
fn optional_finite(node: &SamNode, key: &str) -> Result<f64, SamError> {
    if node.get(key).is_some() {
        finite(node, key)
    } else {
        Ok(0.0)
    }
}

/// Reads one `FORCEBLOCK` of node `nr`; `None` for a disabled block.
fn read_force(block: &SamNode, nr: u32) -> Result<Option<NodalForce>, ImportError> {
    if optional_finite(block, "DISABLED")? != 0.0 {
        // Disabled forces do not act in SAM, so skipping them is faithful.
        return Ok(None);
    }
    let unsupported = |what: String| SamError::Unsupported { what, line: block.line };
    let ednr: u32 = block.num("EDNR")?;
    if ednr != nr {
        return Err(unsupported(format!("FORCEBLOCK EDNR {ednr} in NODE {nr}")).into());
    }
    if optional_finite(block, "ISTORQUE")? != 0.0 {
        return Err(unsupported("external torque".to_owned()).into());
    }
    let any = block.get("FORCEANYARG").ok_or_else(|| SamError::Missing {
        key: "FORCEANYARG".to_owned(),
        line: block.line,
    })?;
    if any.get("FNAME").is_some_and(|n| !n.value.is_empty()) {
        return Err(unsupported("force file".to_owned()).into());
    }
    let argtype: u32 = any.num("ARGTYPE")?;
    let prop = any
        .get("ARGPROP")
        .map(|n| n.value.clone())
        .ok_or_else(|| SamError::Missing {
            key: "ARGPROP".to_owned(),
            line: any.line,
        })?;
    let arg = match (argtype, prop.as_str()) {
        (2, "X") => ForceArg::NodeX(any.num("EDNR")?),
        (2, "Y") => ForceArg::NodeY(any.num("EDNR")?),
        _ => {
            return Err(
                unsupported(format!("force argument ARGTYPE {argtype} ARGPROP {prop}")).into(),
            );
        }
    };
    let table = any.get("TABLE").ok_or_else(|| SamError::Missing {
        key: "TABLE".to_owned(),
        line: any.line,
    })?;
    let mut rows = Vec::with_capacity(table.rows.len());
    for (index, text) in table.rows.iter().enumerate() {
        let bad = || SamError::BadNumber {
            key: "TABLE".to_owned(),
            line: table.line + 1 + index,
        };
        let numbers: Vec<f64> = text
            .split_whitespace()
            .map(|t| t.parse::<f64>().map_err(|_| bad()))
            .collect::<Result<_, _>>()?;
        let row: [f64; 3] = numbers.try_into().map_err(|_| bad())?;
        rows.push(row);
    }
    Ok(Some(NodalForce {
        node: nr,
        arg,
        table: ForceTable::new(rows)?,
    }))
}

/// Fails on any `FORCEBLOCK` that is not a direct child of a `DOFLIST/NODE`.
/// `in_doflist` tells whether `node` is a direct child of the top-level `DOFLIST`.
fn reject_stray_forceblocks(node: &SamNode, in_doflist: bool) -> Result<(), SamError> {
    let is_doflist = node.key == "DOFLIST";
    let is_node_of_doflist = in_doflist && node.key == "NODE";
    for child in &node.children {
        if child.key == "FORCEBLOCK" && !is_node_of_doflist {
            return Err(SamError::Unsupported {
                what: "FORCEBLOCK outside a node".to_owned(),
                line: child.line,
            });
        }
        reject_stray_forceblocks(child, is_doflist)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use mc_core::analyze;

    /// Hand-written slider-crank in the layout of the format description.
    fn slider_crank_text(element: &str, curve: &str) -> String {
        format!(
            "SAMASCII
802
BUILDNRREAD:8.0.0.1
CREATORTYPE_PRO
 ELEMGROUPS:
  ELEMGRP:
   ELEMLIST:
    BEAM:
     NR:1
     N1:1
     N2:2
     A1:1001
     A2:1002
     ENDOBJ
    BEAM:
     NR:2
     N1:2
     N2:3
     A1:1003
     A2:1004
     ENDOBJ
    {element}:
     NR:3
     A1:1005
     A2:1001
     FIXED:0
     ENDOBJ
    ENDOBJ
   ENDOBJ
  ENDOBJ
DOFLIST:
 FIRSTANGLENR:1000
 NODE:
  NR:1
  FIXAXIS:3
  X:0
  Y:0.05
 NODE:
  NR:2
  FIXAXIS:0
  X:0.0877582561890373
  Y:0.0979425538604203
 NODE:
  NR:3
  FIXAXIS:2
  X:0.4
  Y:0
 ANGLE:
  NR:1001
  FIXAXIS:0
  A:0.5
 ANGLE:
  NR:1002
  FIXAXIS:0
  A:0.5
 ANGLE:
  NR:1003
  FIXAXIS:0
  A:-0.3
 ANGLE:
  NR:1004
  FIXAXIS:0
  A:-0.3
 ANGLE:
  NR:1005
  FIXAXIS:4
  A:0
INPUT:
 MAXIM:10
 MOTIONBLOCK:
  EDNR:3
  AXIS:0
  KIND:1
  CURVELIST:
  {curve}:
   NSTEPS:36
   CTIME:1
   MOTION:6.283185307179586
   ENDOBJ
WCS:
 DEFAULT:
ENDSAM
"
        )
    }

    #[test]
    fn imports_minimal_slider_crank() {
        let text = slider_crank_text("ROTMOTOR", "CLINEAR");
        let mech = import_sam(text.as_bytes()).unwrap();
        assert_eq!(mech.elements.len(), 3);
        assert_eq!(mech.inputs.len(), 1);
        assert_eq!(mech.inputs[0].element_nr, 3);
        let traj = analyze(&mech).unwrap();
        assert!(traj.failure.is_none(), "failure: {:?}", traj.failure);
        assert_eq!(traj.states.len(), 37);
    }

    #[test]
    fn gear_is_unsupported() {
        let text = slider_crank_text("GEAR", "CLINEAR");
        let line = text.lines().position(|l| l == "    GEAR:").unwrap() + 1;
        match import_sam(text.as_bytes()).unwrap_err() {
            ImportError::Sam(SamError::Unsupported { what, line: got }) => {
                assert!(what.contains("GEAR"), "what = {what}");
                assert_eq!(got, line);
            }
            other => panic!("unexpected error {other:?}"),
        }
    }

    #[test]
    fn csinq_is_unsupported() {
        let text = slider_crank_text("ROTMOTOR", "CSINQ");
        let line = text.lines().position(|l| l == "  CSINQ:").unwrap() + 1;
        match import_sam(text.as_bytes()).unwrap_err() {
            ImportError::Sam(SamError::Unsupported { what, line: got }) => {
                assert!(what.contains("CSINQ"), "what = {what}");
                assert_eq!(got, line);
            }
            other => panic!("unexpected error {other:?}"),
        }
    }

    const CURVE_FLAT: &str =
        "  CLINEAR:\n   NSTEPS:36\n   CTIME:1\n   MOTION:6.283185307179586\n   ENDOBJ\n";

    fn line_of(text: &str, needle: &str) -> usize {
        text.lines().position(|l| l == needle).unwrap() + 1
    }

    fn sam_err(text: &str) -> SamError {
        match import_sam(text.as_bytes()).unwrap_err() {
            ImportError::Sam(e) => e,
            other => panic!("unexpected error {other:?}"),
        }
    }

    #[test]
    fn rejects_non_positive_ctime() {
        let text = slider_crank_text("ROTMOTOR", "CLINEAR").replace("CTIME:1", "CTIME:0");
        let line = line_of(&text, "   CTIME:0");
        assert_eq!(
            sam_err(&text),
            SamError::BadNumber { key: "CTIME".into(), line }
        );
    }

    #[test]
    fn rejects_zero_nsteps() {
        let text = slider_crank_text("ROTMOTOR", "CLINEAR").replace("NSTEPS:36", "NSTEPS:0");
        let line = line_of(&text, "   NSTEPS:0");
        assert_eq!(
            sam_err(&text),
            SamError::BadNumber { key: "NSTEPS".into(), line }
        );
    }

    #[test]
    fn kind_2_is_unsupported() {
        let text = slider_crank_text("ROTMOTOR", "CLINEAR").replace("KIND:1", "KIND:2");
        let line = line_of(&text, "  KIND:2");
        assert_eq!(
            sam_err(&text),
            SamError::Unsupported { what: "input KIND 2".into(), line }
        );
    }

    #[test]
    fn missing_doflist_is_reported() {
        match sam_err("SAMASCII\n802\nENDSAM\n") {
            SamError::Missing { key, .. } => assert_eq!(key, "DOFLIST"),
            other => panic!("unexpected error {other:?}"),
        }
    }

    #[test]
    fn curves_nested_in_curvelist_are_read() {
        let flat = slider_crank_text("ROTMOTOR", "CLINEAR");
        let nested = flat.replace(
            CURVE_FLAT,
            "   CLINEAR:\n    NSTEPS:36\n    CTIME:1\n    MOTION:6.283185307179586\n    ENDOBJ\n",
        );
        assert_ne!(flat, nested);
        let mech = import_sam(nested.as_bytes()).unwrap();
        assert_eq!(mech.inputs[0].law.segments.len(), 1);
    }

    #[test]
    fn curves_beside_curvelist_under_kind_are_read() {
        // Layout of real files: CURVELIST and the curves are both children of KIND.
        let flat = slider_crank_text("ROTMOTOR", "CLINEAR");
        let real = flat.replace(
            &format!("  CURVELIST:\n{CURVE_FLAT}"),
            "   CURVELIST:\n   CLINEAR:\n    NSTEPS:36\n    CTIME:1\n    MOTION:6.283185307179586\n    ENDOBJ\n",
        );
        assert_ne!(flat, real);
        let mech = import_sam(real.as_bytes()).unwrap();
        assert_eq!(mech.inputs[0].law.segments.len(), 1);
    }

    #[test]
    fn motionblock_without_curves_is_reported() {
        let flat = slider_crank_text("ROTMOTOR", "CLINEAR");
        let text = flat.replace(CURVE_FLAT, "");
        assert_ne!(flat, text);
        let line = line_of(&text, " MOTIONBLOCK:");
        assert_eq!(
            sam_err(&text),
            SamError::Missing { key: "CLINEAR".into(), line }
        );
    }

    #[test]
    fn nonzero_axis_is_unsupported() {
        let text = slider_crank_text("ROTMOTOR", "CLINEAR").replace("
  AXIS:0
", "
  AXIS:1
");
        let line = line_of(&text, "  AXIS:1");
        assert_eq!(
            sam_err(&text),
            SamError::Unsupported { what: "input AXIS 1".into(), line }
        );
    }

    #[test]
    fn missing_axis_defaults_to_zero() {
        let text = slider_crank_text("ROTMOTOR", "CLINEAR").replace("  AXIS:0\n", "");
        assert!(import_sam(text.as_bytes()).is_ok());
    }

    #[test]
    fn rejects_non_finite_coordinates() {
        for (key, line_text, bad) in [
            ("X", "  X:0.4", "  X:NaN"),
            ("Y", "  Y:0.05", "  Y:inf"),
            ("A", "  A:0.5", "  A:-inf"),
        ] {
            let text = slider_crank_text("ROTMOTOR", "CLINEAR");
            let line = line_of(&text, line_text);
            let text = text.replacen(line_text, bad, 1);
            assert_eq!(
                sam_err(&text),
                SamError::BadNumber { key: key.into(), line },
                "key {key}"
            );
        }
    }

    #[test]
    fn caps_nsteps() {
        let text = slider_crank_text("ROTMOTOR", "CLINEAR").replace("NSTEPS:36", "NSTEPS:1000001");
        let line = line_of(&text, "   NSTEPS:1000001");
        assert_eq!(
            sam_err(&text),
            SamError::BadNumber { key: "NSTEPS".into(), line }
        );
        let ok = slider_crank_text("ROTMOTOR", "CLINEAR").replace("NSTEPS:36", "NSTEPS:1000000");
        assert!(import_sam(ok.as_bytes()).is_ok());
    }
}

#[cfg(test)]
mod load_tests {
    use super::*;

    /// Same hand-written slider-crank as in the kinematic tests (made-up numbers).
    fn base() -> String {
        let curve = "CLINEAR";
        format!(
            "SAMASCII
802
BUILDNRREAD:8.0.0.1
CREATORTYPE_PRO
 ELEMGROUPS:
  ELEMGRP:
   ELEMLIST:
    BEAM:
     NR:1
     N1:1
     N2:2
     A1:1001
     A2:1002
     ENDOBJ
    BEAM:
     NR:2
     N1:2
     N2:3
     A1:1003
     A2:1004
     ENDOBJ
    ROTMOTOR:
     NR:3
     A1:1005
     A2:1001
     FIXED:0
     ENDOBJ
    ENDOBJ
   ENDOBJ
  ENDOBJ
DOFLIST:
 FIRSTANGLENR:1000
 NODE:
  NR:1
  FIXAXIS:3
  X:0
  Y:0.05
 NODE:
  NR:2
  FIXAXIS:0
  X:0.0877582561890373
  Y:0.0979425538604203
 NODE:
  NR:3
  FIXAXIS:2
  X:0.4
  Y:0
 ANGLE:
  NR:1001
  FIXAXIS:0
  A:0.5
 ANGLE:
  NR:1002
  FIXAXIS:0
  A:0.5
 ANGLE:
  NR:1003
  FIXAXIS:0
  A:-0.3
 ANGLE:
  NR:1004
  FIXAXIS:0
  A:-0.3
 ANGLE:
  NR:1005
  FIXAXIS:4
  A:0
INPUT:
 MAXIM:10
 MOTIONBLOCK:
  EDNR:3
  AXIS:0
  KIND:1
  CURVELIST:
  {curve}:
   NSTEPS:36
   CTIME:1
   MOTION:6.283185307179586
   ENDOBJ
WCS:
 DEFAULT:
ENDSAM
"
        )
    }

    /// Text of a `FORCEBLOCK` for node 3 with the given overrides.
    fn force_block(disabled: u32, istorque: u32, argtype: u32, argprop: &str) -> String {
        format!(
            "  FORCEBLOCK:
   EDNR:3
   ISTORQUE:{istorque}
   ARGTYPE:2
   DISABLED:{disabled}
   FORCEANYARG:
    FNAME:
    ARGTYPE:{argtype}
    EDNR:2
    ARGPROP:{argprop}
    TABLE:2
     0 1 0
     2 5 0.5
"
        )
    }

    /// Inserts `block` after the last line of node 3 (`  Y:0`).
    fn with_node3(block: &str) -> String {
        let text = base();
        let marker = "  Y:0\n";
        assert!(text.contains(marker));
        text.replacen(marker, &format!("{marker}{block}"), 1)
    }

    fn line_of(text: &str, needle: &str) -> usize {
        text.lines().position(|l| l == needle).unwrap() + 1
    }

    fn unsupported(text: &str) -> (String, usize) {
        match import_project(text.as_bytes()).unwrap_err() {
            ImportError::Sam(SamError::Unsupported { what, line }) => (what, line),
            other => panic!("unexpected error {other:?}"),
        }
    }

    #[test]
    fn imports_node_mass_and_gravity() {
        let text = with_node3("  MASS:\n   MASSVALUE:7.5\n").replace(
            "WCS:\n",
            "GRAVITY:\n VALUE:9.5\n ANGLE:-1.5\n POSX:1\n POSY:2\nWCS:\n",
        );
        let project = import_project(text.as_bytes()).unwrap();
        assert_eq!(project.loads.node_masses, vec![NodeMass { node: 3, mass: 7.5 }]);
        assert_eq!(project.loads.gravity, Some(Gravity { value: 9.5, angle: -1.5 }));
    }

    #[test]
    fn imports_beam_inertia() {
        let text = base().replacen(
            "     N2:2\n",
            "     N2:2\n     M:3\n     XL:0.4\n     J:0.02\n",
            1,
        );
        let project = import_project(text.as_bytes()).unwrap();
        assert_eq!(
            project.loads.element_inertias,
            vec![ElementInertia { element: 1, mass: 3.0, cog: 0.4, inertia: 0.02 }]
        );
    }

    #[test]
    fn imports_node_force_table() {
        let text = with_node3(&force_block(0, 0, 2, "X"));
        let project = import_project(text.as_bytes()).unwrap();
        let forces = &project.loads.forces;
        assert_eq!(forces.len(), 1);
        assert_eq!(forces[0].node, 3);
        assert_eq!(forces[0].arg, ForceArg::NodeX(2));
        assert_eq!(forces[0].table.rows(), &[[0.0, 1.0, 0.0], [2.0, 5.0, 0.5]]);
    }

    #[test]
    fn disabled_force_is_skipped() {
        let text = with_node3(&force_block(1, 0, 2, "X"));
        let project = import_project(text.as_bytes()).unwrap();
        assert!(project.loads.forces.is_empty());
    }

    #[test]
    fn torque_block_is_unsupported() {
        let text = with_node3(&force_block(0, 1, 2, "X"));
        let line = line_of(&text, "  FORCEBLOCK:");
        let (what, got) = unsupported(&text);
        assert!(what.contains("torque"), "what = {what}");
        assert_eq!(got, line);
    }

    #[test]
    fn force_arg_element_angle_is_unsupported() {
        let text = with_node3(&force_block(0, 0, 1, "A1"));
        let (what, _) = unsupported(&text);
        assert_eq!(what, "force argument ARGTYPE 1 ARGPROP A1");
    }

    #[test]
    fn forceblock_outside_node_is_unsupported() {
        let block = "     FORCEBLOCK:\n      EDNR:1\n";
        let text = base().replacen("     N2:2\n", &format!("     N2:2\n{block}"), 1);
        let line = line_of(&text, "     FORCEBLOCK:");
        let (what, got) = unsupported(&text);
        assert_eq!(what, "FORCEBLOCK outside a node");
        assert_eq!(got, line);
    }

    #[test]
    fn hinge_inertia_is_unsupported() {
        let text = base().replacen("     FIXED:0\n", "     FIXED:0\n     J1:0.5\n", 1);
        let (what, _) = unsupported(&text);
        assert!(what.contains("J1"), "what = {what}");
    }

    #[test]
    fn slider_cog_is_unsupported() {
        let text = base()
            .replacen("    BEAM:\n     NR:2\n", "    SLIDER:\n     NR:2\n", 1)
            .replacen("     N2:3\n", "     N2:3\n     M:1\n     XL:0.5\n", 1);
        let (what, _) = unsupported(&text);
        assert!(what.contains("XL"), "what = {what}");
    }

    #[test]
    fn negative_mass_is_a_load_error() {
        let text = with_node3("  MASS:\n   MASSVALUE:-1\n");
        assert!(matches!(
            import_project(text.as_bytes()).unwrap_err(),
            ImportError::Loads(LoadError::InvalidValue { .. })
        ));
    }

    #[test]
    fn import_sam_ignores_loads() {
        let text = with_node3(&force_block(0, 1, 2, "X"));
        assert!(import_sam(text.as_bytes()).is_ok());
    }
}
