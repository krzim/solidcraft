//! Contracts and result types for `cam.simulate` (in `cmd/cam_sim.rs`): a written program run
//! through a model of the stock and what it leaves held against the part, and the same check
//! as `cam.job`'s `simulate` option. docs/cam-jobs.md describes both.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use super::cam::CamJob;

/// Simulate a G-code file removing material from the job's stock and compare what is left with
/// the part: gouges (the part cut by more than the tolerance), rapids that cut material, the
/// material left above the part, and cuts at the tool's full width. Reports; never refuses a
/// program that fails (see `passed`).
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CamSimulateParams {
    /// The G-code file, in the job's work coordinates.
    pub path: String,
    /// The job the program was written for (machine, tools, part bodies, origin, tolerance), as
    /// `cam.job` takes it. Give this or `job_path`; with neither, the open job (`cam.job.open`).
    #[serde(default)]
    pub job: Option<CamJob>,
    /// A `.json` file holding the job.
    #[serde(default)]
    pub job_path: Option<String>,
    /// The size of a cell of the stock's grid in mm (on a 4th axis, along X and round the
    /// stock). Default: five times the job's tolerance, coarser when that would be more than
    /// four million cells; at most twenty million cells.
    #[serde(default)]
    pub resolution: Option<f64>,
    /// Where to write the stock the program leaves as a binary STL file (its top surface, or
    /// its round surface on a 4th axis), for a preview. Default: not written.
    #[serde(default)]
    pub stock_stl: Option<String>,
}

/// What `cam.simulate` returns, and `cam.job`'s `simulation`.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct CamSimulateReply {
    /// The G-code file simulated (empty inside a `cam.job` reply).
    pub path: String,
    /// The stock the simulation started from.
    pub stock: SimStockReport,
    /// The cell size used, mm.
    pub resolution: f64,
    /// Cells in the stock's grid.
    pub cells: usize,
    /// The part cut below its surface.
    pub gouge: SimGougeReport,
    /// Rapids that cut more than the tolerance into the stock: a crash on the machine.
    pub rapids_in_material: Vec<RapidInMaterial>,
    /// Material left above the part where the program cut.
    pub leftover: LeftoverReport,
    /// Material removed, mm³.
    pub removed_volume: f64,
    /// Time at feed as the controller times it, dwells included, minutes.
    pub minutes: f64,
    /// Cutting moves at the tool's full width (slots).
    pub slot_moves: usize,
    /// No gouge and no rapid in material.
    pub passed: bool,
    /// What fails, one line each.
    pub failures: Vec<String>,
    pub warnings: Vec<String>,
    /// How the stock was found and the part tessellated.
    pub notes: Vec<String>,
    /// The STL file of the stock, when asked for.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub stock_stl: Option<String>,
    /// In the warn parameter-check mode: what the call's parameters got wrong.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub param_warnings: Vec<String>,
}

/// The stock before the program, in work coordinates.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct SimStockReport {
    /// `box` (3-axis) or `cylinder` (about work X, a 4th axis).
    pub kind: String,
    /// A box's lowest and highest corners, mm.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub min: Option<[f64; 3]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max: Option<[f64; 3]>,
    /// A cylinder's extent along X and radius, mm.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub x_range: Option<[f64; 2]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub radius: Option<f64>,
    /// Whether it was taken from the part (its box, or the cylinder through its farthest point
    /// from the A axis) because the job gives none.
    pub derived: bool,
}

/// Gouges found by the simulation.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct SimGougeReport {
    /// The deepest cut into the part, mm (0: none beyond the tolerance).
    pub worst: f64,
    /// Cells cut into the part by more than the tolerance.
    pub cells: usize,
}

/// A rapid that cut material.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct RapidInMaterial {
    /// The G-code line, from 1.
    pub line: usize,
    /// Material it cut, mm³.
    pub volume: f64,
    /// How deep into the stock, mm.
    pub depth: f64,
}

/// Material left above the part.
#[derive(Debug, Default, Serialize, JsonSchema)]
pub struct LeftoverReport {
    /// Along Z (along the radius on a 4th axis), over the cells the program cut, mm.
    pub max: f64,
    pub mean: f64,
    /// Cells the program cut.
    pub cells: usize,
}

#[cfg(test)]
#[path = "cam_sim_tests.rs"]
mod tests;
