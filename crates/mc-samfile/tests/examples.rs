//! Integration tests against the SAM example projects.
//!
//! The example files are not part of the repository. They are read from the
//! directory in `SAM_EXAMPLES_DIR` (default: the SAM installation folder) and
//! the tests skip themselves when it does not exist.

use std::path::PathBuf;

use mc_core::analyze;
use mc_samfile::import_sam;

fn examples_dir() -> Option<PathBuf> {
    let dir = std::env::var_os("SAM_EXAMPLES_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from(r"C:\Users\Public\Documents\Sam\Examples"));
    if dir.is_dir() {
        Some(dir)
    } else {
        eprintln!("skipping: SAM examples directory {} not found", dir.display());
        None
    }
}

#[test]
fn slcr_matches_analytic() {
    let Some(dir) = examples_dir() else { return };
    let bytes = std::fs::read(dir.join("SLCR.SAM")).unwrap();
    let mech = import_sam(&bytes).unwrap();
    let traj = analyze(&mech).unwrap();
    assert!(traj.failure.is_none(), "failure: {:?}", traj.failure);

    let pos = |s: &mc_core::State, nr: u32| {
        let (x, y) = mech.node(nr).unwrap();
        (s.q[x.0], s.q[y.0])
    };
    let init = &traj.states[0];
    let (x1, y1) = pos(init, 1);
    let (x2, y2) = pos(init, 2);
    let (x3, y3) = pos(init, 3);
    let r = (x2 - x1).hypot(y2 - y1);
    let l = (x3 - x2).hypot(y3 - y2);
    let theta0 = (y2 - y1).atan2(x2 - x1);
    let law = &mech.inputs[0].law;

    // Pick the rod branch that reproduces the initial slider position.
    let sign = if x3 >= x2 { 1.0 } else { -1.0 };
    for s in &traj.states {
        let (u, _, _) = law.eval(s.t);
        let th = theta0 + u;
        let w = y1 + r * th.sin() - y3;
        let expected = x1 + r * th.cos() + sign * (l * l - w * w).sqrt();
        let (px, py) = pos(s, 3);
        assert!(
            (px - expected).abs() < 1e-9,
            "t={} x3={} expected={}",
            s.t,
            px,
            expected
        );
        assert!((py - y3).abs() < 1e-9);
    }
}

#[test]
fn phase1_examples_analyze() {
    let Some(dir) = examples_dir() else { return };
    let mut failures = Vec::new();
    for name in [
        "SLCR.SAM",
        "6BAR.SAM",
        "CRRK.SAM",
        "ORTHO.SAM",
        "DYAD.sam",
        "SCS.sam",
        "TRFR.SAM",
        "QKRT.SAM",
        "DXFDEMO (detached).SAM",
    ] {
        let result = std::fs::read(dir.join(name))
            .map_err(|e| e.to_string())
            .and_then(|bytes| import_sam(&bytes).map_err(|e| format!("import: {e}")))
            .and_then(|mech| analyze(&mech).map_err(|e| format!("analyze: {e}")))
            .and_then(|traj| match traj.failure {
                None => Ok(traj.states.len()),
                Some(f) => Err(format!("failure after {} states: {f}", traj.states.len())),
            });
        match &result {
            Ok(n) => eprintln!("{name}: ok, {n} states"),
            Err(e) => {
                eprintln!("{name}: {e}");
                failures.push(format!("{name}: {e}"));
            }
        }
    }
    assert!(failures.is_empty(), "failing examples: {failures:#?}");
}

/// BED.SAM drives BEAM 1 by elongation (`ELONGATION:1`, input `KIND:1`). The model
/// does not support inputs on beams yet, so the import reports it. When mc-core
/// gains beam-elongation inputs this test must move into `phase1_examples_analyze`.
#[test]
fn bed_beam_elongation_input_is_not_yet_supported() {
    let Some(dir) = examples_dir() else { return };
    let bytes = std::fs::read(dir.join("BED.SAM")).unwrap();
    assert_eq!(
        import_sam(&bytes).unwrap_err(),
        mc_samfile::ImportError::Model(mc_core::ModelError::InputOnNonDrivable(1))
    );
}
