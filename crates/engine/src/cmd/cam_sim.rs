//! `cam.simulate`, and `cam.job`'s `simulate` option: a written program run through a model of
//! the stock (`solidcraft_cam::sim`) as an independent check of the G-code, and what it leaves
//! held against the part. The program is parsed as the controller would read it; the job gives
//! the machine, the tools, the part's bodies (tessellated as `cam.job` does) and the tolerance.
//!
//! A job without a stock starts from one taken from the part: its box on a 3-axis machine, the
//! cylinder about work X through its farthest point from the axis on a 4th axis.

use std::path::{Path, PathBuf};

use serde_json::Value;
use solidcraft_cam::kinematics::Machine;
use solidcraft_cam::sim::{SimParams, SimReport, StockShape, simulate as run_sim, stock_around};
use solidcraft_cam::{Surface, Tool};

use super::{MAX_GCODE_BYTES, TOLERANCE_RANGE, cam_err, finite_point, machine_of, num, parse, read_json, surfaces_of, tools_of, v3};
use crate::params::bad;
use crate::schema::TypedContract;
use crate::schema::cam::CamJob;
use crate::schema::cam_sim::{CamSimulateParams, CamSimulateReply, LeftoverReport, RapidInMaterial, SimGougeReport, SimStockReport};
use crate::{EngineError, Result, Session};

pub(super) const CONTRACT: TypedContract<CamSimulateParams> = TypedContract::new();

/// Most points along a side of the stock's preview mesh.
const STL_SIDE: usize = 1024;
/// Rapids in material listed in a reply.
const LISTED: usize = 100;

/// The job of `job` (`raw` as given) or `job_path`, the folder a job file is in, and whether an
/// operation of it has `gouge_check: false`.
fn load_job(cmd: &str, job: Option<CamJob>, raw: Option<&Value>, job_path: Option<&String>) -> Result<(CamJob, Option<PathBuf>, bool)> {
    match (job, job_path) {
        (Some(_), Some(_)) => Err(bad(cmd, "give `job` or `job_path`, not both")),
        (None, None) => Err(bad(cmd, "give the `job` (an object) or a `job_path` (a .json file)")),
        (Some(j), None) => Ok((j, None, raw.and_then(|v| v.get("operations")).is_some_and(unchecked))),
        (None, Some(path)) => {
            let v = read_json(cmd, path, "job_path")?;
            let deliberate = v.get("operations").is_some_and(unchecked);
            Ok((parse(cmd, &v, &format!("job_path {path}"))?, Path::new(path).parent().map(Path::to_path_buf), deliberate))
        }
    }
}

/// Whether `v` (a job's operations as JSON, an indexed one's inner operation included) has
/// `gouge_check: false` anywhere.
fn unchecked(v: &Value) -> bool {
    match v {
        Value::Object(m) => m.get("gouge_check") == Some(&Value::Bool(false)) || m.values().any(unchecked),
        Value::Array(a) => a.iter().any(unchecked),
        _ => false,
    }
}

/// The stock the program starts from, and whether it was taken from the part.
fn stock_of(cmd: &str, part: &Surface, machine: &Machine) -> Result<(StockShape, bool)> {
    let stock = stock_around(part, machine).map_err(|e| cam_err(cmd, "the stock", e))?;
    Ok((stock, true))
}

fn stock_report(stock: &StockShape, derived: bool) -> SimStockReport {
    match *stock {
        StockShape::Box { min, max } => SimStockReport {
            kind: "box".into(),
            min: Some([min.x, min.y, min.z]),
            max: Some([max.x, max.y, max.z]),
            derived,
            ..SimStockReport::default()
        },
        StockShape::Cylinder { x_min, x_max, radius } => {
            SimStockReport { kind: "cylinder".into(), x_range: Some([x_min, x_max]), radius: Some(radius), derived, ..SimStockReport::default() }
        }
    }
}

/// Simulates `text` for the job's `machine` and `tools` against `part` at `tol`. With
/// `deliberate_gouges` (an operation of the job cuts into the part on purpose) gouges are
/// warned about rather than failed. The reply and the simulation's own report.
pub(super) fn check_program(
    cmd: &str,
    text: &str,
    machine: &Machine,
    tools: &[Tool],
    part: &Surface,
    tol: f64,
    resolution: Option<f64>,
    deliberate_gouges: bool,
) -> Result<(CamSimulateReply, SimReport)> {
    if resolution.is_some_and(|r| !(1e-4..=1e4).contains(&r)) {
        return Err(bad(cmd, "`resolution` must be from 0.0001 to 10000 mm"));
    }
    let (stock, derived) = stock_of(cmd, part, machine)?;
    let report = run_sim(text, machine, tools, &stock, part, &SimParams { tolerance: tol, resolution }).map_err(|e| cam_err(cmd, "simulation", e))?;
    let mut failures = Vec::new();
    let mut warnings = report.warnings.clone();
    if report.gouge_cells > 0 {
        let what =
            format!("the program cuts up to {} mm into the part over {} cells of the simulated stock", num(report.gouge_worst), report.gouge_cells);
        if deliberate_gouges {
            warnings.push(format!("{what} (an operation has `gouge_check: false`, so this is not failed)"));
        } else {
            failures.push(format!("simulation: {what}"));
        }
    }
    if let Some(first) = report.rapids_in_material.first() {
        failures.push(format!(
            "simulation: {} rapids cut material (a crash on the machine), the first at line {}, {} mm deep",
            report.rapids_in_material.len(),
            first.line,
            num(first.depth)
        ));
    }
    let mut notes = vec![if derived {
        match stock {
            StockShape::Box { .. } => "the job gives no stock: the simulation starts from the part's box".to_string(),
            StockShape::Cylinder { radius, .. } => {
                format!(
                    "the job gives no stock: the simulation starts from a cylinder of radius {} mm about work X over the part's length",
                    num(radius)
                )
            }
        }
    } else {
        "the simulation starts from the job's stock".to_string()
    }];
    notes.push(format!("simulated on {} cells of {} mm", report.cells, num(report.cell)));
    if report.rapids_in_material.len() > LISTED {
        notes.push(format!("the first {LISTED} of {} rapids in material are listed", report.rapids_in_material.len()));
    }
    let reply = CamSimulateReply {
        path: String::new(),
        stock: stock_report(&stock, derived),
        resolution: report.cell,
        cells: report.cells,
        gouge: SimGougeReport { worst: report.gouge_worst, cells: report.gouge_cells },
        rapids_in_material: report
            .rapids_in_material
            .iter()
            .take(LISTED)
            .map(|r| RapidInMaterial { line: r.line, volume: r.volume, depth: r.depth })
            .collect(),
        leftover: LeftoverReport { max: report.leftover_max, mean: report.leftover_mean, cells: report.machined_cells },
        removed_volume: report.removed_volume,
        minutes: report.minutes,
        slot_moves: report.removals.iter().filter(|r| r.slot).count(),
        passed: failures.is_empty(),
        failures,
        warnings,
        notes,
        stock_stl: None,
        param_warnings: Vec::new(),
    };
    Ok((reply, report))
}

/// `cam.simulate`.
pub(super) fn simulate(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.simulate";
    let params: CamSimulateParams = parse(cmd, p, "")?;
    if let Some(out) = &params.stock_stl {
        if out.trim().is_empty() {
            return Err(bad(cmd, "a file path must not be empty"));
        }
        if *out == params.path {
            return Err(bad(cmd, "`stock_stl` must not be the G-code file"));
        }
        solidcraft_io::vfs::check(out).map_err(|e| EngineError::Other(e.to_string()))?;
    }
    let (job, base, deliberate) = load_job(cmd, params.job, p.get("job"), params.job_path.as_ref())?;
    let tol = job.tolerance.unwrap_or(0.01);
    if !(tol.is_finite() && (TOLERANCE_RANGE.0..=TOLERANCE_RANGE.1).contains(&tol)) {
        return Err(bad(cmd, "the job's `tolerance` must be from 0.0001 to 1 mm"));
    }
    let origin = job.origin.unwrap_or([0.0; 3]);
    finite_point(&origin, "`origin`").map_err(|e| bad(cmd, e))?;
    let machine = machine_of(cmd, &job)?;
    let tools = tools_of(cmd, &job, base.as_deref())?;
    let bytes = solidcraft_io::vfs::read(&params.path, MAX_GCODE_BYTES).map_err(|e| EngineError::Other(format!("{}: {e}", params.path)))?;
    let text = String::from_utf8_lossy(&bytes);
    let surfaces = surfaces_of(cmd, s, &job, v3(origin), tol)?;
    let (mut reply, report) = check_program(cmd, &text, &machine, &tools, &surfaces.part, tol, params.resolution, deliberate)?;
    reply.path = params.path.clone();
    reply.notes.extend(surfaces.notes);
    if let Some(out) = &params.stock_stl {
        let mesh = report.stock.mesh(STL_SIDE);
        solidcraft_io::vfs::write(out, &solidcraft_io::stl_binary(&[mesh], "stock")).map_err(|e| EngineError::Other(format!("{out}: {e}")))?;
        reply.stock_stl = Some(out.clone());
    }
    serde_json::to_value(reply).map_err(|e| EngineError::Other(e.to_string()))
}
