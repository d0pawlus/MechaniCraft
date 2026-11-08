//! End-to-end tests of the `mcraft` binary.

use std::fs;
use std::path::Path;
use std::process::Output;

use assert_cmd::Command;
use tempfile::TempDir;

/// Hand-written slider-crank in the SAM layout (copied from the import tests).
const SLIDER_CRANK: &str = "SAMASCII
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
  CLINEAR:
   NSTEPS:36
   CTIME:1
   MOTION:6.28318512
   ENDOBJ
WCS:
 DEFAULT:
ENDSAM
";

/// Same mechanism but with a `GEAR` element, which the importer rejects.
const WITH_GEAR: &str = "SAMASCII
802
DOFLIST:
 FIRSTANGLENR:1000
ELEMGROUPS:
  ELEMGRP:
   ELEMLIST:
    GEAR:
     NR:3
     ENDOBJ
    ENDOBJ
   ENDOBJ
  ENDOBJ
ENDSAM
";

fn write_file(dir: &Path, name: &str, text: &str) {
    fs::write(dir.join(name), text).unwrap();
}

fn run(args: &[&str]) -> Output {
    Command::cargo_bin("mcraft").unwrap().args(args).output().unwrap()
}

fn stdout_of(output: &Output) -> String {
    String::from_utf8(output.stdout.clone()).unwrap()
}

fn stderr_of(output: &Output) -> String {
    String::from_utf8(output.stderr.clone()).unwrap()
}

#[test]
fn analyze_writes_csv() {
    let dir = TempDir::new().unwrap();
    let input = dir.path().join("demo (1).sam");
    fs::write(&input, SLIDER_CRANK).unwrap();
    let out = dir.path().join("out.csv");

    let output = run(&["analyze", input.to_str().unwrap(), "-o", out.to_str().unwrap()]);
    assert_eq!(output.status.code(), Some(0), "stderr = {}", stderr_of(&output));

    let csv = fs::read_to_string(&out).unwrap();
    let lines: Vec<&str> = csv.split('\n').collect();
    // 37 states plus header; the file ends with a newline.
    assert_eq!(lines.len(), 39);
    assert_eq!(lines[38], "");
    assert_eq!(
        lines[0],
        "step,t,X1,Y1,VX1,VY1,AX1,AY1,X2,Y2,VX2,VY2,AX2,AY2,X3,Y3,VX3,VY3,AX3,AY3"
    );
    assert!(lines[1].starts_with("0,0.000000000000e0,"), "row = {}", lines[1]);
    assert_eq!(lines[1].split(',').count(), 2 + 6 * 3);
}

#[test]
fn analyze_selected_nodes_are_ascending() {
    let dir = TempDir::new().unwrap();
    let input = dir.path().join("demo.sam");
    fs::write(&input, SLIDER_CRANK).unwrap();

    let output = run(&["analyze", input.to_str().unwrap(), "--nodes", "3,2"]);
    assert_eq!(output.status.code(), Some(0), "stderr = {}", stderr_of(&output));
    let stdout = stdout_of(&output);
    assert_eq!(
        stdout.lines().next().unwrap(),
        "step,t,X2,Y2,VX2,VY2,AX2,AY2,X3,Y3,VX3,VY3,AX3,AY3"
    );
    assert_eq!(stdout.lines().count(), 38);
}

#[test]
fn analyze_unknown_node_fails_with_exit_1() {
    let dir = TempDir::new().unwrap();
    let input = dir.path().join("demo.sam");
    fs::write(&input, SLIDER_CRANK).unwrap();

    let output = run(&["analyze", input.to_str().unwrap(), "--nodes", "99"]);
    assert_eq!(output.status.code(), Some(1));
    assert!(stderr_of(&output).contains("99"), "stderr = {}", stderr_of(&output));
}

#[test]
fn analyze_unsupported_input_fails_with_exit_1() {
    let dir = TempDir::new().unwrap();
    let input = dir.path().join("gear.sam");
    fs::write(&input, WITH_GEAR).unwrap();

    let output = run(&["analyze", input.to_str().unwrap()]);
    assert_eq!(output.status.code(), Some(1));
    assert!(stderr_of(&output).contains("GEAR"), "stderr = {}", stderr_of(&output));
}

#[test]
fn check_reports_unsupported_without_failing() {
    let dir = TempDir::new().unwrap();
    write_file(dir.path(), "gear.sam", WITH_GEAR);

    let output = run(&["check", dir.path().to_str().unwrap()]);
    assert_eq!(output.status.code(), Some(0), "stderr = {}", stderr_of(&output));
    let stdout = stdout_of(&output);
    assert!(stdout.contains("UNSUPPORTED"), "stdout = {stdout}");
    assert!(stdout.contains("1 files: 0 OK, 1 UNSUPPORTED, 0 FAILED"), "stdout = {stdout}");
}

#[test]
fn check_summarises_and_fails_only_on_failed() {
    let dir = TempDir::new().unwrap();
    write_file(dir.path(), "a_slider.SAM", SLIDER_CRANK);
    write_file(dir.path(), "b_gear.sam", WITH_GEAR);
    write_file(dir.path(), "c_broken.sam", "SAMASCII\n802\nENDSAM\n");
    write_file(dir.path(), "notes.txt", "ignored");

    let output = run(&["check", dir.path().to_str().unwrap()]);
    assert_eq!(output.status.code(), Some(1));
    let stdout = stdout_of(&output);
    let lines: Vec<&str> = stdout.lines().collect();
    assert_eq!(lines.len(), 4, "stdout = {stdout}");
    assert!(lines[0].starts_with("OK  a_slider.SAM  37"), "line = {}", lines[0]);
    assert!(lines[1].starts_with("UNSUPPORTED  b_gear.sam  "), "line = {}", lines[1]);
    assert!(lines[2].starts_with("FAILED  c_broken.sam  "), "line = {}", lines[2]);
    assert_eq!(lines[3], "3 files: 1 OK, 1 UNSUPPORTED, 1 FAILED");
}

#[test]
fn analyze_partial_result_exits_with_2() {
    let dir = TempDir::new().unwrap();
    let input = dir.path().join("runaway.sam");
    // An absurd rotation makes Newton fail after a few steps.
    fs::write(&input, SLIDER_CRANK.replace("MOTION:6.28318512", "MOTION:1000000")).unwrap();
    let out = dir.path().join("partial.csv");

    let output = run(&["analyze", input.to_str().unwrap(), "-o", out.to_str().unwrap()]);
    assert_eq!(output.status.code(), Some(2), "stderr = {}", stderr_of(&output));
    assert!(stderr_of(&output).contains("incomplete"), "stderr = {}", stderr_of(&output));

    let csv = fs::read_to_string(&out).unwrap();
    let rows = csv.lines().count();
    assert!(rows >= 2 && rows < 38, "rows = {rows}");
    assert!(csv.starts_with("step,t,X1,Y1"));
}
