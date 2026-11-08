//! Conversion of a parsed SAM project into a [`Mechanism`].
//!
//! Phase 1 supports `BEAM`, `SLIDER`, `HINGE` and `ROTMOTOR` elements, inputs of
//! `KIND:1` and `CLINEAR` curve segments. Everything else is reported as
//! [`SamError::Unsupported`] with the line of the offending block.

use mc_core::{Mechanism, MechanismBuilder, ModelError};
use mc_motion::{InputLaw, Segment};

use crate::tree::{SamError, SamNode, SamTree, parse_tree};

/// Errors produced while importing a SAM project.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum ImportError {
    /// The file could not be read or uses unsupported content.
    #[error(transparent)]
    Sam(#[from] SamError),
    /// The file was read but does not describe a valid mechanism.
    #[error(transparent)]
    Model(#[from] ModelError),
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

fn required<'a>(tree: &'a SamTree, key: &str) -> Result<&'a SamNode, SamError> {
    tree.get(key).ok_or_else(|| SamError::Missing {
        key: key.to_owned(),
        line: 0,
    })
}

fn read_dofs(tree: &SamTree, builder: &mut MechanismBuilder) -> Result<(), SamError> {
    let doflist = required(tree, "DOFLIST")?;
    for child in &doflist.children {
        match child.key.as_str() {
            "NODE" => {
                let nr: u32 = child.num("NR")?;
                let x: f64 = child.num("X")?;
                let y: f64 = child.num("Y")?;
                let fix: u32 = child.num("FIXAXIS")?;
                builder.node(nr, x, y, fix & 1 != 0, fix & 2 != 0);
            }
            "ANGLE" => {
                let nr: u32 = child.num("NR")?;
                let a: f64 = child.num("A")?;
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
        // Real files put the curves next to CURVELIST under MOTIONBLOCK; an
        // indented layout would nest them below KIND, so both are searched.
        let mut segments = Vec::new();
        for curve in block.children.iter().chain(kind.children.iter()) {
            if curve.key == "CURVELIST" || !curve.key.starts_with('C') {
                continue;
            }
            if curve.key != "CLINEAR" {
                return Err(SamError::Unsupported {
                    what: format!("motion {}", curve.key),
                    line: curve.line,
                });
            }
            segments.push(read_linear(curve)?);
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
    if steps < 1 {
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

#[cfg(test)]
mod tests {
    use super::*;
    use mc_core::analyze;

    /// Hand-written slider-crank in the layout of the format description.
    fn slider_crank_text(element: &str, curve: &str) -> String {
        format!(
            "SAMASCII
802
BUILDNRREAD:8.0.1.161
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
   MOTION:6.28318512
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

    #[test]
    fn rejects_non_positive_ctime() {
        let text = slider_crank_text("ROTMOTOR", "CLINEAR").replace("CTIME:1", "CTIME:0");
        assert!(matches!(
            import_sam(text.as_bytes()).unwrap_err(),
            ImportError::Sam(SamError::BadNumber { .. })
        ));
    }

    #[test]
    fn missing_doflist_is_reported() {
        let err = import_sam(b"SAMASCII\n802\nENDSAM\n").unwrap_err();
        assert!(matches!(err, ImportError::Sam(SamError::Missing { .. })));
    }
}
