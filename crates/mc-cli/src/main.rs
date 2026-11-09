//! `mcraft`: command-line front end for the MechaniCraft kinematic core.
//!
//! * `mcraft analyze <FILE> [-o <CSV>] [--nodes 3,2] [--forces]` imports a
//!   `.SAM` file, runs the kinematic analysis and writes a CSV table of node
//!   positions, velocities and accelerations. With `--forces` the loads are
//!   imported too and the kinetostatic columns are appended: driving force and
//!   power per input (`DRV`, `PWR`), reactions on fixed coordinates (`RX`, `RY`,
//!   `RA`) and the normal force of each beam (`N`).
//! * `mcraft check <DIR>` runs the import, kinematics and kinetostatics on every
//!   `.sam` file in a directory and prints one status line per file.
//!
//! Exit codes: `0` success, `1` import, I/O or argument error (or a `FAILED`
//! file in `check`), `2` kinematic or force analysis stopped early (partial
//! CSV was written).

use std::fs::{self, File};
use std::io::{self, BufWriter, Write};
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use anyhow::{Context, Result, anyhow};
use clap::error::ErrorKind;
use clap::{Parser, Subcommand};
use mc_core::{CoordId, CoordLabel, ElementKind, Mechanism, Trajectory, analyze};
use mc_kineto::{ForceTrajectory, kinetostatics};
use mc_samfile::{ImportError, SamError, import_project, import_sam};

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
        /// Import the loads and append driving force, power, reactions and
        /// beam normal forces to the table.
        #[arg(long)]
        forces: bool,
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
        Command::Analyze { file, output, nodes, forces } => {
            analyze_command(&file, output.as_deref(), nodes, forces)
        }
        Command::Check { dir } => check_command(&dir),
    }
}

fn analyze_command(
    file: &Path,
    output: Option<&Path>,
    nodes: Option<Vec<u32>>,
    forces: bool,
) -> Result<ExitCode> {
    let bytes = fs::read(file).with_context(|| format!("cannot read {}", file.display()))?;
    let (mech, loads) = if forces {
        let project =
            import_project(&bytes).with_context(|| format!("cannot import {}", file.display()))?;
        (project.mechanism, Some(project.loads))
    } else {
        (import_sam(&bytes).with_context(|| format!("cannot import {}", file.display()))?, None)
    };
    let selected = select_nodes(&mech, nodes)?;
    let traj = analyze(&mech).context("analysis setup failed")?;
    let force_traj = match &loads {
        Some(loads) => Some(kinetostatics(&mech, loads, &traj).context("kinetostatics failed")?),
        None => None,
    };
    let columns = force_traj.as_ref().map(|_| ForceColumns::new(&mech));
    let extra = match (&columns, &force_traj) {
        (Some(c), Some(ft)) => Some((c, ft)),
        _ => None,
    };

    match output {
        Some(path) => {
            let mut writer = BufWriter::new(
                File::create(path).with_context(|| format!("cannot create {}", path.display()))?,
            );
            write_csv(&mut writer, &traj, &selected, extra)?;
            writer.flush()?;
        }
        None => {
            let stdout = io::stdout();
            let mut writer = BufWriter::new(stdout.lock());
            write_csv(&mut writer, &traj, &selected, extra)?;
            writer.flush()?;
        }
    }

    let mut code = ExitCode::SUCCESS;
    if let Some(err) = &traj.failure {
        eprintln!("mcraft: analysis incomplete after {} steps: {err}", traj.states.len());
        code = ExitCode::from(EXIT_PARTIAL);
    }
    if let Some(ft) = &force_traj {
        if let Some(err) = &ft.failure {
            eprintln!("mcraft: kinetostatics incomplete after {} steps: {err}", ft.states.len());
            code = ExitCode::from(EXIT_PARTIAL);
        }
    }
    Ok(code)
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

/// Where the value of one kinetostatic column comes from.
enum ForceSource {
    Driving(usize),
    Power(usize),
    Reaction(CoordId),
    Normal(u32),
}

/// The kinetostatic columns appended by `--forces`, in output order: per
/// input `DRV`/`PWR`, then reactions on fixed coordinates (nodes ascending by
/// NR with `RX` before `RY`, then angles ascending by NR), then `N` per beam
/// ascending by element NR.
struct ForceColumns {
    cols: Vec<(String, ForceSource)>,
}

impl ForceColumns {
    fn new(mech: &Mechanism) -> Self {
        let mut cols = Vec::new();
        for (i, input) in mech.inputs.iter().enumerate() {
            cols.push((format!("DRV{}", input.element_nr), ForceSource::Driving(i)));
            cols.push((format!("PWR{}", input.element_nr), ForceSource::Power(i)));
        }

        let mut node_cols: Vec<(u32, u8, CoordId)> = Vec::new();
        let mut angle_cols: Vec<(u32, CoordId)> = Vec::new();
        for (idx, coord) in mech.coords.iter().enumerate() {
            if !coord.fixed {
                continue;
            }
            match coord.label {
                CoordLabel::NodeX(nr) => node_cols.push((nr, 0, CoordId(idx))),
                CoordLabel::NodeY(nr) => node_cols.push((nr, 1, CoordId(idx))),
                CoordLabel::Angle(nr) => angle_cols.push((nr, CoordId(idx))),
            }
        }
        node_cols.sort_unstable_by_key(|&(nr, axis, _)| (nr, axis));
        angle_cols.sort_unstable_by_key(|&(nr, _)| nr);
        for (nr, axis, id) in node_cols {
            let name = if axis == 0 { format!("RX{nr}") } else { format!("RY{nr}") };
            cols.push((name, ForceSource::Reaction(id)));
        }
        for (nr, id) in angle_cols {
            cols.push((format!("RA{nr}"), ForceSource::Reaction(id)));
        }

        let mut beams: Vec<u32> = mech
            .elements
            .iter()
            .filter(|e| matches!(e.kind, ElementKind::Beam { .. }))
            .map(|e| e.nr)
            .collect();
        beams.sort_unstable();
        for nr in beams {
            cols.push((format!("N{nr}"), ForceSource::Normal(nr)));
        }
        ForceColumns { cols }
    }

    /// Value of every column at `step`. Mode 0 of a beam is always an equation
    /// of the mode system, so the zero fallback for a missing stress cannot
    /// occur; it only avoids a panic.
    fn values(&self, forces: &ForceTrajectory, step: usize) -> Vec<f64> {
        let state = &forces.states[step];
        self.cols
            .iter()
            .map(|(_, source)| match *source {
                ForceSource::Driving(i) => state.driving[i],
                ForceSource::Power(i) => state.power[i],
                ForceSource::Reaction(id) => state.reactions[id.0],
                ForceSource::Normal(nr) => forces.stress(step, nr, 0).unwrap_or(0.0),
            })
            .collect()
    }
}

/// Writes the CSV table: a header line, then one row per solver state (with
/// force columns: per state present in both the kinematic and force results).
fn write_csv(
    out: &mut dyn Write,
    traj: &Trajectory,
    nodes: &[(u32, CoordId, CoordId)],
    forces: Option<(&ForceColumns, &ForceTrajectory)>,
) -> Result<()> {
    let mut header = String::from("step,t");
    for (nr, _, _) in nodes {
        for name in ["X", "Y", "VX", "VY", "AX", "AY"] {
            header.push_str(&format!(",{name}{nr}"));
        }
    }
    if let Some((columns, _)) = forces {
        for (name, _) in &columns.cols {
            header.push(',');
            header.push_str(name);
        }
    }
    out.write_all(header.as_bytes())?;
    out.write_all(b"\n")?;

    let rows = match forces {
        Some((_, ft)) => traj.states.len().min(ft.states.len()),
        None => traj.states.len(),
    };
    for (step, state) in traj.states.iter().enumerate().take(rows) {
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
        if let Some((columns, ft)) = forces {
            for v in columns.values(ft, step) {
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

/// Import, kinematics and kinetostatics; `OK` only when all three succeed.
fn classify(bytes: &[u8]) -> Outcome {
    let project = match import_project(bytes) {
        Err(ImportError::Sam(SamError::Unsupported { what, line })) => {
            return Outcome::Unsupported(format!("{what} at line {line}"));
        }
        Err(err) => return Outcome::Failed(err.to_string()),
        Ok(project) => project,
    };
    let traj = match analyze(&project.mechanism) {
        Err(err) => return Outcome::Failed(err.to_string()),
        Ok(traj) => traj,
    };
    if let Some(err) = &traj.failure {
        return Outcome::Failed(err.to_string());
    }
    match kinetostatics(&project.mechanism, &project.loads, &traj) {
        Err(err) => Outcome::Failed(format!("kinetostatics: {err}")),
        Ok(forces) => match forces.failure {
            Some(err) => Outcome::Failed(format!("kinetostatics: {err}")),
            None => Outcome::Ok(traj.states.len()),
        },
    }
}
