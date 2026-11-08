//! `mcraft`: command-line front end for the MechanismCraft kinematic core.
//!
//! * `mcraft analyze <FILE> [-o <CSV>] [--nodes 3,2]` imports a `.SAM` file,
//!   runs the kinematic analysis and writes a CSV table of node positions,
//!   velocities and accelerations.
//! * `mcraft check <DIR>` runs the import and analysis on every `.sam` file in a
//!   directory and prints one status line per file.
//!
//! Exit codes: `0` success, `1` import, I/O or argument error (or a `FAILED`
//! file in `check`), `2` analysis stopped early (partial CSV was written).

use std::fs::{self, File};
use std::io::{self, BufWriter, Write};
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use anyhow::{Context, Result, anyhow};
use clap::error::ErrorKind;
use clap::{Parser, Subcommand};
use mc_core::{CoordId, CoordLabel, Mechanism, Trajectory, analyze};
use mc_samfile::{ImportError, SamError, import_sam};

/// Exit code when the analysis stopped before the end of the motion.
const EXIT_PARTIAL: u8 = 2;

#[derive(Parser)]
#[command(name = "mcraft", version, about = "Planar mechanism kinematics from SAM files")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Analyse a `.SAM` file and write node kinematics as CSV.
    Analyze {
        /// Path to the `.SAM` file.
        file: PathBuf,
        /// Write the CSV to this file instead of stdout.
        #[arg(short, long, value_name = "CSV")]
        output: Option<PathBuf>,
        /// Comma-separated node numbers to export (default: all nodes).
        #[arg(long, value_name = "NR,NR,...", value_delimiter = ',')]
        nodes: Option<Vec<u32>>,
    },
    /// Import and analyse every `.sam` file in a directory.
    Check {
        /// Directory to scan (not recursive).
        dir: PathBuf,
    },
}

fn main() -> ExitCode {
    // Usage errors must exit 1, not clap's default 2, because 2 means "partial CSV".
    let cli = match Cli::try_parse() {
        Ok(cli) => cli,
        Err(err) => {
            let _ = err.print();
            return match err.kind() {
                ErrorKind::DisplayHelp | ErrorKind::DisplayVersion => ExitCode::SUCCESS,
                _ => ExitCode::from(1),
            };
        }
    };
    match run(cli) {
        Ok(code) => code,
        Err(err) => {
            eprintln!("mcraft: {err:#}");
            ExitCode::from(1)
        }
    }
}

fn run(cli: Cli) -> Result<ExitCode> {
    match cli.command {
        Command::Analyze { file, output, nodes } => analyze_command(&file, output.as_deref(), nodes),
        Command::Check { dir } => check_command(&dir),
    }
}

fn analyze_command(file: &Path, output: Option<&Path>, nodes: Option<Vec<u32>>) -> Result<ExitCode> {
    let bytes = fs::read(file).with_context(|| format!("cannot read {}", file.display()))?;
    let mech = import_sam(&bytes).with_context(|| format!("cannot import {}", file.display()))?;
    let selected = select_nodes(&mech, nodes)?;
    let traj = analyze(&mech).context("analysis setup failed")?;

    match output {
        Some(path) => {
            let mut writer = BufWriter::new(
                File::create(path).with_context(|| format!("cannot create {}", path.display()))?,
            );
            write_csv(&mut writer, &traj, &selected)?;
            writer.flush()?;
        }
        None => {
            let stdout = io::stdout();
            let mut writer = BufWriter::new(stdout.lock());
            write_csv(&mut writer, &traj, &selected)?;
            writer.flush()?;
        }
    }

    match &traj.failure {
        None => Ok(ExitCode::SUCCESS),
        Some(err) => {
            eprintln!("mcraft: analysis incomplete after {} steps: {err}", traj.states.len());
            Ok(ExitCode::from(EXIT_PARTIAL))
        }
    }
}

/// Resolves the requested node numbers to `(nr, x, y)` coordinate ids in
/// ascending node order. Without a request, all nodes of the mechanism are used.
fn select_nodes(mech: &Mechanism, requested: Option<Vec<u32>>) -> Result<Vec<(u32, CoordId, CoordId)>> {
    let mut nrs: Vec<u32> = match requested {
        Some(list) => list,
        None => mech
            .coords
            .iter()
            .filter_map(|c| match c.label {
                CoordLabel::NodeX(nr) => Some(nr),
                _ => None,
            })
            .collect(),
    };
    nrs.sort_unstable();
    nrs.dedup();

    nrs.into_iter()
        .map(|nr| {
            let (x, y) = mech.node(nr).ok_or_else(|| anyhow!("unknown node {nr}"))?;
            Ok((nr, x, y))
        })
        .collect()
}

/// Writes the CSV table: a header line, then one row per solver state.
fn write_csv(out: &mut dyn Write, traj: &Trajectory, nodes: &[(u32, CoordId, CoordId)]) -> Result<()> {
    let mut header = String::from("step,t");
    for (nr, _, _) in nodes {
        for name in ["X", "Y", "VX", "VY", "AX", "AY"] {
            header.push_str(&format!(",{name}{nr}"));
        }
    }
    out.write_all(header.as_bytes())?;
    out.write_all(b"\n")?;

    for (step, state) in traj.states.iter().enumerate() {
        let mut row = format!("{step},{:.12e}", state.t);
        for (_, x, y) in nodes {
            let values = [
                state.q[x.0],
                state.q[y.0],
                state.qd[x.0],
                state.qd[y.0],
                state.qdd[x.0],
                state.qdd[y.0],
            ];
            for v in values {
                row.push_str(&format!(",{v:.12e}"));
            }
        }
        out.write_all(row.as_bytes())?;
        out.write_all(b"\n")?;
    }
    Ok(())
}

/// Result of checking one file.
enum Outcome {
    Ok(usize),
    Unsupported(String),
    Failed(String),
}

fn check_command(dir: &Path) -> Result<ExitCode> {
    let mut files = sam_files(dir)?;
    files.sort_by(|a, b| a.file_name().cmp(&b.file_name()));

    let stdout = io::stdout();
    let mut out = stdout.lock();
    let (mut ok, mut unsupported, mut failed) = (0usize, 0usize, 0usize);
    for path in &files {
        let name = path.file_name().unwrap_or_default().to_string_lossy().into_owned();
        let outcome = match fs::read(path) {
            Ok(bytes) => classify(&bytes),
            Err(err) => Outcome::Failed(err.to_string()),
        };
        let (status, detail) = match outcome {
            Outcome::Ok(states) => {
                ok += 1;
                ("OK", format!("{states} states"))
            }
            Outcome::Unsupported(what) => {
                unsupported += 1;
                ("UNSUPPORTED", what)
            }
            Outcome::Failed(msg) => {
                failed += 1;
                ("FAILED", msg)
            }
        };
        writeln!(out, "{status}  {name}  {detail}").context("cannot write check report")?;
    }

    writeln!(
        out,
        "{} files: {ok} OK, {unsupported} UNSUPPORTED, {failed} FAILED",
        files.len()
    )
    .context("cannot write check summary")?;
    out.flush().context("cannot write check report")?;
    Ok(if failed > 0 { ExitCode::from(1) } else { ExitCode::SUCCESS })
}

/// Lists the entries of `dir` (non-recursive) whose extension is `sam`,
/// compared case-insensitively. Only directories are skipped; an entry whose
/// file type cannot be read is kept, so that reading it reports a FAILED line
/// instead of vanishing silently.
fn sam_files(dir: &Path) -> Result<Vec<PathBuf>> {
    let entries =
        fs::read_dir(dir).with_context(|| format!("cannot read directory {}", dir.display()))?;
    let mut files = Vec::new();
    for entry in entries {
        let entry = entry.with_context(|| format!("cannot list directory {}", dir.display()))?;
        let path = entry.path();
        let is_sam = path
            .extension()
            .and_then(|e| e.to_str())
            .is_some_and(|e| e.eq_ignore_ascii_case("sam"));
        let is_dir = matches!(entry.file_type(), Ok(ft) if ft.is_dir());
        if is_sam && !is_dir {
            files.push(path);
        }
    }
    Ok(files)
}

fn classify(bytes: &[u8]) -> Outcome {
    match import_sam(bytes) {
        Err(ImportError::Sam(SamError::Unsupported { what, .. })) => Outcome::Unsupported(what),
        Err(err) => Outcome::Failed(err.to_string()),
        Ok(mech) => match analyze(&mech) {
            Err(err) => Outcome::Failed(err.to_string()),
            Ok(traj) => match traj.failure {
                Some(err) => Outcome::Failed(err.to_string()),
                None => Outcome::Ok(traj.states.len()),
            },
        },
    }
}
