//! Manufacturing (CAM) foundation for SolidCraft: cutting tools, toolpaths, 2D contour and
//! pocket milling of planar regions, G-code output through declarative post processors, and a
//! G-code reader that checks what was written against the toolpaths it was written from.
//!
//! Everything here works in a setup's work coordinates: millimetres, feeds in mm/min. The 2D
//! operations have the tool axis along +Z. The crate knows nothing of documents or the B-rep
//! kernel; the engine hands it regions and surfaces.
//!
//! Beyond 2D: [`Surface`] holds a part's triangles, and the drop-cutter ([`drop_z`],
//! [`drop_along`]) lowers a flat, ball, bull nose or tapered (chamfer) cutter onto them along any axis. 3D
//! roughing and finishing are built on it, and so are the multi-axis toolpaths ([`Toolpath5`]:
//! the tip with the tool axis) of indexed (3+2), 4-axis rotary and simultaneous 5-axis
//! machining, which machine kinematics turn into linear and rotary axis positions.
//!
//! The region offsetting is our own: every boundary piece is offset, corners that open away
//! from the offset side get a joining arc, the pieces are cut where they cross, and a piece is
//! kept when its midpoint is no closer to the boundary than the offset distance. Lines and
//! arcs stay lines and arcs, so walls come out as straight moves and true arcs.
#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]
#![forbid(unsafe_code)]

mod chamfer;
mod check;
pub mod collision;
mod cutter;
mod dropcutter;
mod frame;
mod gcode;
pub mod gcode5;
pub mod indexed;
pub mod kinematics;
mod multiaxis;
mod offset;
mod ops;
mod path;
mod piece;
mod post;
mod readback;
pub mod readback5;
pub mod rotary;
pub mod strategies5;
mod surface;
pub mod surface_ops;
mod tool;

pub use chamfer::{ChamferGeometry, ChamferParams, DEFAULT_TIP_OFFSET, chamfer, chamfer_geometry};
pub use check::verify;
pub use cutter::{Cone, Cutter};
pub use dropcutter::{drop_along, drop_triangle, drop_z};
pub use frame::{Frame, angle_about, rotate, rotate_about};
pub use gcode::{Operation, Program, emit, parse, work_offset_code};
pub use multiaxis::{Move5, Operation5, PathStats5, Pose, Program5, Toolpath5, slerp, verify5};
pub use offset::offset_regions;
pub use ops::{ContourParams, Cut, Feeds, Heights, PocketParams, Side, contour, pocket};
pub use path::{Move, PathStats, Toolpath, arc_sweep};
pub use post::{CommentStyle, LineNumbers, Post, SafeRetract, Units};
pub use readback::{Readback, allowed_deviation, readback};
pub use rotary::{
    DEFAULT_MAX_ANGLE_STEP, RotaryParams, RotaryPattern, RotaryRoughingParams, radial, rotary_finish, rotary_pose, rotary_roughing, wrap,
    wrap_with_step,
};
pub use surface::{Surface, barycentric, capsule_meets_box, nearest_on_triangle, nearest_segments, ray_triangle, segment_triangle_distance};
pub use surface_ops::{Boundary, MAX_GRID_NODES, MAX_SAMPLES, ParallelParams, RoughingParams, WaterlineParams, parallel, roughing, waterline};
pub use tool::{Envelope, HolderSection, Tool, ToolKind};

#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum CamError {
    #[error("invalid value: {0}")]
    Invalid(String),
    #[error("not supported yet: {0}")]
    Unsupported(String),
    #[error("too large: {0}")]
    TooLarge(String),
    /// A toolpath that breaks a safety rule (a rapid through material, a cut below the
    /// bottom). Operations check their own output, so this reports a defect rather than bad input.
    #[error("unsafe toolpath: {0}")]
    Unsafe(String),
    #[error("G-code line {0}: {1}")]
    Parse(usize, String),
}

pub type Result<T> = std::result::Result<T, CamError>;

pub(crate) fn invalid<T>(why: impl Into<String>) -> Result<T> {
    Err(CamError::Invalid(why.into()))
}

#[cfg(test)]
mod chamfer_tests;
#[cfg(test)]
mod collision_tests;
#[cfg(test)]
mod foundation_tests;
#[cfg(test)]
mod gcode5_tests;
#[cfg(test)]
mod kinematics_tests;
#[cfg(test)]
mod rotary_tests;
#[cfg(test)]
mod strategies5_tests;
#[cfg(test)]
mod surface_ops_tests;
#[cfg(test)]
mod tests;
