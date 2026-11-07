//! `cam.simulate`, and `cam.job`'s `simulate` option: a written program run through a model of
//! the stock (`solidcraft_cam::sim`) as an independent check of the G-code, and what it leaves
//! held against the part. The program is parsed as the controller would read it; the job gives
//! the machine, the tools, the part's bodies (tessellated as `cam.job` does) and the tolerance.
//!
//! The simulation starts from the job's `stock` when its shape suits the machine (a box on a
//! 3-axis machine, a bar about X on a 4th axis). Otherwise, and for a job without a stock, it
//! starts from one taken from the part: its box on a 3-axis machine, the cylinder about work X
//! through its farthest point from the axis on a 4th axis.

use serde_json::Value;
use solidcraft_cam::helical::{ISO_EXTERNAL_DEPTH, ISO_INTERNAL_DEPTH, thread_core};
use solidcraft_cam::kinematics::Machine;
use solidcraft_cam::sim::{SimParams, SimReport, StockShape, simulate as run_sim, stock_around};
use solidcraft_cam::stock::Stock;
use solidcraft_cam::{Surface, Tool, ToolKind};

use super::{MAX_GCODE_BYTES, TOLERANCE_RANGE, cam_err, finite_point, machine_of, num, parse, surfaces_of, tools_of, v3};
use crate::params::bad;
use crate::schema::TypedContract;
use crate::schema::cam::{CamJob, OperationSpec};
use crate::schema::cam_agent::ToolRef;
use crate::schema::cam_sim::{CamSimulateParams, CamSimulateReply, LeftoverReport, RapidInMaterial, SimGougeReport, SimStockReport};
use crate::{EngineError, Result, Session};

pub(super) const CONTRACT: TypedContract<CamSimulateParams> = TypedContract::new();

/// Most points along a side of the stock's preview mesh.
const STL_SIDE: usize = 1024;
/// Rapids in material listed in a reply.
const LISTED: usize = 100;

/// `tools` as the simulation cuts with them: a thread mill that a thread milling operation uses
/// is taken as the core inside its teeth (`cores`, by tool number; the smallest of a number's),
/// as the gouge check takes it, since the thread it cuts is not in the model.
pub(super) fn with_thread_cores(tools: &[Tool], cores: &[Tool]) -> Vec<Tool> {
    tools
        .iter()
        .map(|t| {
            let core = cores.iter().filter(|c| c.number == t.number).min_by(|a, b| a.diameter.total_cmp(&b.diameter));
            match core {
                Some(c) if t.kind == ToolKind::ThreadMill => c.clone(),
                _ => t.clone(),
            }
        })
        .collect()
}

/// The cores of the thread mills the job's `thread_mill` operations name by number, each as
/// deep as its operation's thread.
pub(super) fn job_thread_cores(job: &CamJob, tools: &[Tool]) -> Vec<Tool> {
    job.operations
        .iter()
        .filter_map(|op| {
            let OperationSpec::ThreadMill(o) = op else { return None };
            let ToolRef::Number(n) = o.tool else { return None };
            let tool = tools.iter().find(|t| t.number == n && t.kind == ToolKind::ThreadMill)?;
            let iso = if o.internal.unwrap_or(true) { ISO_INTERNAL_DEPTH } else { ISO_EXTERNAL_DEPTH };
            thread_core(tool, o.thread_depth.unwrap_or(o.pitch * iso)).ok()
        })
        .collect()
}

/// Whether `v` (a job's operations as JSON, an indexed one's inner operation included) has
/// `gouge_check: false` anywhere.
pub(super) fn unchecked(v: &Value) -> bool {
    match v {
        Value::Object(m) => m.get("gouge_check") == Some(&Value::Bool(false)) || m.values().any(unchecked),
        Value::Array(a) => a.iter().any(unchecked),
        _ => false,
    }
}

/// The stock the program starts from, and whether it was taken from the part: the job's
/// `stock` when the simulation can model it on this machine, else one round the part (with a
/// note saying why).
pub(super) fn stock_of(
    cmd: &str,
    job_stock: Option<&Stock>,
    part: &Surface,
    machine: &Machine,
    notes: &mut Vec<String>,
) -> Result<(StockShape, bool)> {
    let rotary = !machine.rotaries.is_empty();
    match job_stock {
        Some(&Stock::Box { min, max }) if !rotary => return Ok((StockShape::Box { min, max }, false)),
        // A square or rectangular bar with the A axis through it.
        Some(&Stock::Box { min, max }) if min.y < 0.0 && max.y > 0.0 && min.z < 0.0 && max.z > 0.0 => {
            return Ok((StockShape::Box { min, max }, false));
        }
        Some(&Stock::Cylinder { radius, x_min, x_max }) if rotary => return Ok((StockShape::Cylinder { x_min, x_max, radius }, false)),
        Some(Stock::Box { .. }) => {
            notes.push("the simulation models a bar the A axis runs through on a 4th axis, not the job's box stock off the axis".into())
        }
        Some(Stock::Cylinder { .. }) => notes.push("the simulation models a box on a 3-axis machine, not the job's bar stock".into()),
        None => {}
    }
    let stock = stock_around(part, machine).map_err(|e| cam_err(cmd, "the stock", e))?;
    Ok((stock, true))
}

pub(super) fn stock_report(stock: &StockShape, derived: bool) -> SimStockReport {
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
    job_stock: Option<&Stock>,
    tol: f64,
    resolution: Option<f64>,
    deliberate_gouges: bool,
) -> Result<(CamSimulateReply, SimReport)> {
    if resolution.is_some_and(|r| !(1e-4..=1e4).contains(&r)) {
        return Err(bad(cmd, "`resolution` must be from 0.0001 to 10000 mm"));
    }
    let mut stock_notes = Vec::new();
    let (stock, derived) = stock_of(cmd, job_stock, part, machine, &mut stock_notes)?;
    let report = run_sim(text, machine, tools, &stock, part, &SimParams { tolerance: tol, resolution }).map_err(|e| cam_err(cmd, "simulation", e))?;
    let mut failures = Vec::new();
    let mut warnings = report.warnings.clone();
    if report.moves == 0 {
        // Nothing checked is not a pass.
        failures.push(
            "simulation: no move of the program could be followed (none comes after every axis of the machine, its rotary included, is \
             written)"
                .to_string(),
        );
    }
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
    notes.extend(stock_notes);
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
    if let Some(out) = &params.stock_heights {
        if out.trim().is_empty() || *out == params.path {
            return Err(bad(cmd, "`stock_heights` must be a file path other than the G-code file"));
        }
        solidcraft_io::vfs::check(out).map_err(|e| EngineError::Other(e.to_string()))?;
    }
    if let Some(out) = &params.stock_stl {
        if out.trim().is_empty() {
            return Err(bad(cmd, "a file path must not be empty"));
        }
        if *out == params.path {
            return Err(bad(cmd, "`stock_stl` must not be the G-code file"));
        }
        solidcraft_io::vfs::check(out).map_err(|e| EngineError::Other(e.to_string()))?;
    }
    // Read as `cam.job` reads it: its profile merged in, its machine file and tool libraries read.
    let loaded = super::agent::load_job_of(cmd, p, params.job.is_some(), params.job_path.as_ref())?;
    let (job, base, deliberate) = (loaded.job, loaded.base, loaded.unchecked);
    let tol = job.tolerance.unwrap_or(0.01);
    if !(tol.is_finite() && (TOLERANCE_RANGE.0..=TOLERANCE_RANGE.1).contains(&tol)) {
        return Err(bad(cmd, "the job's `tolerance` must be from 0.0001 to 1 mm"));
    }
    let origin = job.origin.unwrap_or([0.0; 3]);
    finite_point(&origin, "`origin`").map_err(|e| bad(cmd, e))?;
    let machine = machine_of(cmd, &job)?;
    let tools = tools_of(cmd, &job, base.as_deref())?;
    let tools = with_thread_cores(&tools, &job_thread_cores(&job, &tools));
    let bytes = solidcraft_io::vfs::read(&params.path, MAX_GCODE_BYTES).map_err(|e| EngineError::Other(format!("{}: {e}", params.path)))?;
    let text = String::from_utf8_lossy(&bytes);
    let surfaces = surfaces_of(cmd, s, &job, v3(origin), tol)?;
    let mut stock_notes = Vec::new();
    let job_stock = super::cam_stock::stock_of(cmd, s, &job, base.as_deref(), &surfaces, tol, &mut stock_notes)?;
    let (mut reply, report) = check_program(cmd, &text, &machine, &tools, &surfaces.part, job_stock.as_ref(), tol, params.resolution, deliberate)?;
    reply.path = params.path.clone();
    reply.notes.extend(surfaces.notes);
    reply.notes.extend(stock_notes);
    reply.notes.extend(loaded.notes);
    if let Some(out) = &params.stock_stl {
        export_stock(out, &report)?;
        reply.stock_stl = Some(out.clone());
    }
    if let Some(out) = &params.stock_heights {
        let solidcraft_cam::sim::StockGrid::Heights { x0, y0, dx, dy, nx, ny, bottom, top, heights } = report.stock else {
            return Err(bad(cmd, "`stock_heights` is for a 3-axis job: a 4th axis's stock is not a height map"));
        };
        let place = super::placement_of(cmd, s, &job)?;
        let file = super::cam_stock::StockHeightsFile {
            format: super::cam_stock::STOCK_HEIGHTS_FORMAT.into(),
            map: solidcraft_cam::adaptive::HeightMap { x0, y0, dx, dy, nx, ny, heights },
            bottom,
            top,
            origin: [place.origin.x, place.origin.y, place.origin.z],
            axes: place.axes.map(|a| a.map(|v| [v.x, v.y, v.z])),
        };
        let text = serde_json::to_vec(&file).map_err(|e| EngineError::Other(e.to_string()))?;
        super::write_file(out, &text).map_err(|e| EngineError::Other(format!("{out}: {e}")))?;
        reply.notes.push(format!("the stock left is written as a height map to {out}"));
    }
    serde_json::to_value(reply).map_err(|e| EngineError::Other(e.to_string()))
}

/// The stock the simulation left, written to `out` as a binary STL file (export: the stock's
/// display mesh, never a decision).
fn export_stock(out: &str, report: &SimReport) -> Result<()> {
    let mesh = report.stock.mesh(STL_SIDE);
    super::write_file(out, &solidcraft_io::stl_binary(&[mesh], "stock")).map_err(|e| EngineError::Other(format!("{out}: {e}")))
}
