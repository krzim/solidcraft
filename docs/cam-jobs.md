# CAM jobs

A CAM job turns a design's bodies into G-code for one machine. It is a JSON file kept next to
the design (or an object passed inline); nothing of it is saved in the `.sc` file. Four commands
work with jobs, from scripts, the CLI and MCP alike:

| Command | What it does |
| --- | --- |
| `cam.job` | Plans every operation, writes the program through the job's post, reads it back and checks it, and writes the G-code (and an HTML preview) |
| `cam.posts` | Lists the built-in posts, or returns one as JSON to copy into a post file |
| `cam.check` | Reads a G-code file and reports lengths, time, the rotary range, inverse-time blocks and the first line it cannot read |
| `cam.simulate` | Runs a G-code file through a model of the stock and compares what is left with the part: gouges, rapids through material, leftover, slots (see "Stock-removal simulation") |

Units are millimetres and mm/min. Angles in a job are degrees (`a_deg`, `angle_deg`, …).

## `cam.job`

```json
{"job_path": "C:/parts/bracket.cam.json", "output": "C:/parts/bracket.tap", "preview": "C:/parts/bracket.html"}
```

| Key | Meaning |
| --- | --- |
| `job` | The job as an object. Give this or `job_path`. |
| `job_path` | A `.json` file holding the job. Relative `post_path` and `tool_library_path` in it are read from the file's folder. |
| `output` | The G-code file. Required unless `dry_run`. |
| `preview` | An HTML page of the part and every toolpath: self-contained (no network), one view per operation and one for the whole job, orbit and zoom, a slider that plays the moves, rapids dashed. |
| `dry_run` | Plan and check, write nothing; the reply says whether the checks pass instead of refusing. |
| `allow_unsafe` | Write the program even when a check fails. |
| `simulate` | Also run the program as written through the stock-removal simulation (below): a gouge or a rapid through material fails the job like the other checks, and the reply carries `simulation`. |

Every file is read and written through the session's file access, so an MCP server's
`--allow-path` folders apply; every path is checked before anything is computed or written.

### The checks

The command never writes a program that fails one of these unless `allow_unsafe` is given:

- **Read-back.** The program as written is read back through the machine's kinematics and
  compared with the toolpaths, both ways round: the tip within the post's precision and
  tolerances (`readback.allowed_tip`), the tool axis within its angle precision.
- **Gouges.** Every cutting move of every operation is sampled and the cutter (shrunk by the
  job's tolerance) must not meet the part or a fixture. An operation that cuts what the model
  does not show (an engraving) sets `gouge_check: false`; the reply lists it under
  `gouge.skipped`.
- **Holder.** For tools with holder data (`stickout` above the flutes, `shank_diameter`,
  `holder`), the shank and holder must not touch the part or a fixture along a cutting move.

Without `dry_run` or `allow_unsafe`, a failure refuses the call with every failed check in the
message. With `dry_run`, the reply carries `passed: false` and `failures`.

### The reply

`operations` (each with `name`, `kind`, `tool`, `moves`, `cut_length`, `cut_minutes`,
`max_tilt_deg`, `a_deg` when indexed, `gouge`, `holder_collisions`, `warnings`), `program_lines`,
`program_head` (the first lines), `readback`, `gouge`, `holder_collisions` (`null` when no tool
has holder data), `passed`, `failures`, `dry_run`, `outputs` (`gcode`, `preview`) and `notes`
(how the bodies were tessellated, a post that writes no work offset, an output extension other
than the post's).

## The job

```json
{
  "name": "bracket",
  "machine": "4axis_a",
  "post_path": "my-mill.post.json",
  "origin": [0, 0, 0],
  "tolerance": 0.01,
  "tools": [
    {"number": 1, "kind": "flat_endmill", "diameter": 6, "flute_length": 20, "spindle_rpm": 12000, "feed": 800, "plunge_feed": 200},
    {"number": 2, "kind": "ball_endmill", "diameter": 4, "flute_length": 12, "spindle_rpm": 14000, "feed": 900, "plunge_feed": 250}
  ],
  "operations": [
    {"name": "top pocket", "type": "pocket", "tool": 1, "polygon": [[-10, -5], [10, -5], [10, 5], [-10, 5]],
     "heights": {"clearance": 30, "retract": 20, "top": 15, "bottom": 10}, "stepdown": 1.5},
    {"name": "side pocket", "type": "indexed", "a_deg": 90,
     "operation": {"name": "side pocket", "type": "pocket", "tool": 1, "polygon": [[-5, -5], [5, -5], [5, 5], [-5, 5]],
                   "heights": {"clearance": 30, "retract": 20, "top": 15, "bottom": 12}}},
    {"name": "barrel finish", "type": "rotary_finish", "tool": 2, "pattern": {"type": "lines", "angle_step_deg": 5},
     "clearance": 30, "retract": 22, "range": [-20, 20]}
  ]
}
```

Every object refuses keys it does not know, so a misspelt key is an error, never a setting
silently left at its default. That holds for the tools, the machine and the post too.

| Key | Meaning |
| --- | --- |
| `name` | The program's name, in its header. Default: the design's name. |
| `machine` | `"3axis"`, `"4axis_a"` (a rotary table about +X, the A axis, through work zero), or a whole machine as the CAM crate's `Machine` JSON (`name`, `rotaries: [{letter, direction, center, carrier, limits?, reversed?}]`, `linear_limits?`, `work_origin?`; radians). |
| `a_reversed` | With `4axis_a`: the controller's positive A turns the table the other way. |
| `a_limits_deg` | With `4axis_a`: `[lowest, highest]` A in degrees. Default: it turns without end. |
| `post` | A built-in post (`cam.posts` lists them): `mach3_4axis`, `linuxcnc_4axis`, `grblhal_4axis`, `rs274_5axis`, `rs274_tcp`. |
| `post_path` | A post as a JSON file: the CAM crate's `Post5` (`cam.posts {"name": …}` returns one to start from). This is how a machine maker's own post is matched: copy the closest built-in post and change its header, tool change, spindle, comments, decimals, inverse time and rotary settings until its output matches. |
| `work_offset` | 1 to 6 (G54 to G59), for posts whose header writes `{work_offset}`. Default: 1. |
| `origin` | The design point that is work zero, `[x, y, z]`. Default: the design's origin. |
| `bodies` | The part's bodies by name. Default: every visible body not in `fixtures`. |
| `fixtures` | Bodies the tool must stay clear of (vises, clamps): in the gouge and holder checks only, never machined. |
| `tolerance` | 0.0001 to 1 mm (default 0.01): how far a toolpath may be from the part, and how deep a cut into it counts as a gouge. |
| `tools` | The tools, as the CAM crate's `Tool` JSON (below). Give this or `tool_library_path`. |
| `tool_library_path` | A JSON file holding an array of tools. |
| `operations` | 1 to 200 operations, run and written in order (below). |

### Coordinates

Work coordinates are the design's coordinates less `origin`. On a 4th axis the rotary axis is
work X through work zero: model the part with its turning axis along the design's X through the
origin, or give `origin` as a point on that axis. A rotary toolpath's tip at angle θ and radius
ρ is at `(x, ρ sin θ, ρ cos θ)` with `A = θ`: A = +90° brings the work's +Y side up under the
tool.

### Tessellation

Each body is tessellated from its exact B-rep surfaces with a chord tolerance of half the job's
tolerance (`solidcraft_kernel::MeshTolerance::chord`, the tolerance-driven path the mesh exports
are built on; the export presets `standard`/`fine`/`finest` are a share of the body's size and
so are not used). The facets then lie within half the tolerance of the true surfaces, which
leaves the other half for the toolpath. A mesh body (an imported STL, 3MF or OBJ) is used as it
is; the reply's notes say so.

### Tools

`number`, `kind` (`flat_endmill`, `ball_endmill`, `bull_nose`, `drill`), `diameter`,
`flute_length`, and optional `description`, `flutes`, `spindle_rpm`, `feed`, `plunge_feed`,
`stepdown`, `stepover`, `corner_radius` (bull nose), `shank_diameter`, `stickout`, `holder:
[{diameter, length}]`. An operation without `spindle_rpm` or `feeds` takes the tool's; its
stepdown and stepover default to the tool's too.

### Operations

Every operation has `name`, `type`, `tool` (a tool's `number`), and optionally `spindle_rpm`,
`feeds: {cut, plunge}`, `coolant` (default false) and `gouge_check` (default true). `heights`
are `{clearance, retract, top, bottom}` in the operation's coordinates.

| `type` | Keys (defaults) |
| --- | --- |
| `contour` | `sketch` (id or name) + `profiles?` (indices; default every drawn profile) or `polygon: [[x, y], …]`; `heights`; `stepdown` (tool's, else half its diameter); `side`: `outside` \| `inside` \| `on` (outside); `climb` (true); `stock_to_leave` (0); `ramp_angle_deg` (0: plunge); `tolerance` (the job's) |
| `pocket` | `sketch` + `profiles?` or `polygon`; `heights`; `stepdown`; `stepover` (tool's, else 40% of its diameter); `climb`; `stock_to_leave`; `ramp_angle_deg`; `tolerance` |
| `parallel` | `heights` (`bottom` is the floor); `stepover` (tool's, else 10% of its diameter); `angle_deg` (0); `boundary?: {rect?: [[x, y], [x, y]], polygon?}` (the part's extent); `stock_to_leave` (0); `step` (a quarter of the diameter); `tolerance`; `zigzag` (true) |
| `roughing` | `heights` (`top` is the stock's top); `stepdown`; `stepover`; `angle_deg`; `boundary`; `stock_to_leave` (0.25); `step`; `tolerance`; `zigzag` |
| `waterline` | `heights` (levels from `top` to `bottom`); `stepdown` (tool's, else a quarter of its diameter); `boundary`; `stock_to_leave` (0); `step`; `tolerance`; `climb` (true) |
| `indexed` | `a_deg` and `operation`: any of the five above, run with A held at `a_deg` (3+1). Only these two keys; the inner operation carries the name, tool and feeds. |
| `rotary_finish` | `pattern`: `{type: "lines", angle_step_deg, zigzag?}` or `{type: "helix", pitch}`; `clearance` and `retract` (radii from the A axis); `floor` (0); `range: [min x, max x]` (the part's extent); `step`; `tolerance`; `stock_to_leave` (0); `max_angle_step_deg` (2) |
| `rotary_roughing` | As `rotary_finish`, with `stock_radius`, `stepdown` (in radius) and `stock_to_leave` (0.25) |
| `wrap` | `radius` (of the cylinder the path is drawn on); `points: [[x, y, z], …]` (x along the axis, y the arc length round it, z the depth: 0 on the cylinder, negative into it) or `sketch` + `curve` (a curve id; its sketch x and y are taken as x and arc length) with `depth` (0); `clearance` (5) and `retract` (1) above the cylinder; `max_angle_step_deg` (2) |

The 3-axis operations (`contour` to `waterline`) run with every rotary at zero; on `3axis` and
`4axis_a` that is work coordinates. Inside `indexed` their heights, polygons and boundaries are
in the frame A = `a_deg` brings under the tool: its Z is the tool axis, so at A = 90° the frame's
Z is work +Y. A sketch used as a boundary must lie on a plane square to the operation's tool axis
and facing it (for a plain 3-axis operation, a plane facing +Z), in the design's root component.
Indexed and 3-axis operations are written with arcs kept (`G2`/`G3`); rotary and wrapped
operations as short cuts, each turning A by at most `max_angle_step_deg`, in inverse time where
the post says so.

## `cam.posts`

`{}` lists the built-in posts with a one-line description and their other names; `{"name":
"mach3_4axis"}` returns that post's JSON. Save it as a file, change it, and point a job's
`post_path` at it. Keep a machine-specific post outside the repository.

## `cam.check`

`{"path": "part.tap", "rapid_rate": 3000}` reads the file and returns `lines`, `motion_blocks`,
`rapid_length`, `feed_length`, `cut_minutes` (as the controller times it: inverse-time blocks by
their `F`), `rapid_minutes` (with `rapid_rate`), `arcs`, `g93_blocks`, `rotary_ranges` (each
rotary letter's lowest and highest angle), `bounds` and `errors` (the first line the reader
cannot read). It reads the absolute-coordinate G-code SolidCraft's posts write; programs from
elsewhere are read on a best-effort basis and stop at the first block it cannot follow
(incremental coordinates, for one).

## Stock-removal simulation

`cam.simulate` checks a written program independently of the toolpaths it came from: it reads
the G-code as the controller would, sweeps each tool through a model of the stock along every
block (rapids included), and compares what is left with the part. `cam.job` runs the same check
on the program it has just written when given `simulate: true`.

```json
{"path": "C:/parts/bracket.tap", "job_path": "C:/parts/bracket.cam.json", "resolution": 0.05, "stock_stl": "C:/parts/bracket-stock.stl"}
```

| Key | Meaning |
| --- | --- |
| `path` | The G-code file, in the job's work coordinates. |
| `job` / `job_path` | The job the program was written for: its machine, tools (by `T` number; `T… M6` loads one), part bodies, `origin` and `tolerance`. |
| `resolution` | The cell size in mm. Default: five times the tolerance, coarser when that would be more than 4 million cells. More than 20 million cells is refused. |
| `stock_stl` | Write the stock the program leaves as a binary STL file (its top surface, or its round surface on a 4th axis), for a preview. |

The reply: `stock` (`kind`: `box` or `cylinder`, its size, and `derived`), `resolution`,
`cells`, `gouge: {worst, cells}` (the deepest cut into the part beyond the tolerance, and how many
cells), `rapids_in_material: [{line, volume, depth}]` (rapids that cut more than the tolerance
into the stock: a crash on the machine), `leftover: {max, mean, cells}` (material left above the
part over the cells the program cut), `removed_volume` (mm³), `minutes` (at feed, as the
controller times it), `slot_moves` (cuts at the tool's full width, also listed in `warnings`:
keep their depth and feed low on a light machine), `passed` (no gouge and no rapid in material),
`failures`, `warnings`, `notes` and `stock_stl`. A failed simulation is reported, never refused;
inside `cam.job` its failures join the job's. When an operation of the job has `gouge_check:
false` (an engraving), the simulation's gouges are warned about, not failed.

**The stock.** A job without a stock starts from one taken from the part: on a 3-axis machine
the part's box, on a 4th axis the cylinder about work X through the part's farthest point from
the axis, over the part's length. A program whose rapids are safe for a block-shaped part may
then cut that round bar at its retract height, which is what the bar would do on the machine.

**3-axis.** The stock is a heightfield over the box: one top height per cell, at the cell's
centre. Every block lowers each cell to the tool's lowest point over it along the whole move,
found exactly, not at sampled positions: along a straight move the tool's height over a cell is
convex in the position along the move, so its least value is found in closed form (flat end
mill, ball, drill point at 118°) or by bisection on its slope to rounding (bull nose). A level arc
is exact as well; an arc that also moves in Z (a helix) is followed as chords at most a quarter
of the tolerance inside it. The tool is its cutting end with its diameter carried on up, so a cut
deeper than the flutes still removes material.

**4th axis (A about X).** The stock is a grid over (x, A) holding how far out the material
reaches along the ray from the axis. A block that turns A while X moves is a helix in the work
frame, so the tool is placed at points along each block (every axis moving evenly, as written),
close enough that the cusps between them are at most half the tolerance: at most √(4·r·tol)
apart for a tool with corner radius r (a ball's is its radius), √(4·R·tol) for a flat end mill of
radius R, tol / slope for a drill point, counting both the tip's travel and the cutting end's as
the axis turns. At each point every ray near the tool is cut where it first enters the tool,
when the tool reaches out of the material along it. A ray holds one stretch of material, so a cut
that crosses a ray inside the material without reaching out of it (an indexed pocket in the
part's side does this to the rays at a slant through it) is left out of that ray and counted in
a warning; `cam.job`'s own gouge check, against the exact cutter, still covers those cuts.

**Against the part.** Each cell's part height (or radius) is where a ray from above (from
outside) first meets the part's facets. A cell cut below the part by more than the tolerance is
measured again in 3D, as the distance from the bottom and the middle of the cut column to the
part's surface, so a cut beside a steep wall within the tolerance of the wall is not a gouge.
Leftover is measured along Z (along the ray), so material beside a steep wall counts its full
height. The position is lost at a return home or a move in machine coordinates (`G28`, `G30`,
`G53`); the next block is taken as coming straight down the tool axis from above the stock,
where those moves leave the tool, and so is the program's first.

**Time.** The grid is split into bands of rows (of angles on a 4th axis), each running the whole
program on a thread of its own, one per core up to sixteen (one on the web); the result does not
depend on how many. A 50 × 50 mm dome finished with a 6 mm ball in parallel passes 0.5 mm apart
(about 21,000 blocks) simulates on a million cells (0.05 mm) in about 7 seconds on one thread and
under a second on eight; a 4th-axis finish on a 16 mm long, 20 mm diameter barrel, 1° apart, on
414,000 cells in about 3 seconds on one thread and under a second on eight.
