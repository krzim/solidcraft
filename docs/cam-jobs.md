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
| `setup_sheet` | The setup sheet for the machine operator: a `.html` page (self-contained, print styles) or a `.md` file. See [Setup sheet](#setup-sheet). |
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
- **Rapids.** Every `G0` of the program as written, the stock and the turns of a 4th axis
  included (see "Stock, facing and rapid safety" below).
- **Travel.** With a machine whose JSON has `linear_limits`, every position is within them.
- **Operation checks** that are part of planning, each refusing the job with a message that
  names the operation: a drill's rapid down to the R height over the part round a hole (see
  [Drilling](#drilling)), a face down into the stock or below it (see
  [Facing](#facing)), an A turn between operations that would swing the part into the tool (see
  [The setup check](#the-setup-check)), and every tool used where it cannot cut (see
  [Which tool for which operation](#which-tool-for-which-operation)).
- **Simulation**, with `simulate: true`: the program as written removes material from the
  stock, and a gouge or a rapid through material fails the job (see
  [Stock-removal simulation](#stock-removal-simulation)).

With `machine_rates` the reply also times the program as the machine runs it and warns about
blocks its controller would slow (see [Machine rates](#machine-rates)); those are warnings, not
failures.

Without `dry_run` or `allow_unsafe`, a failure refuses the call with every failed check in the
message. With `dry_run`, the reply carries `passed: false` and `failures`.

### The reply

`operations` (each with `name`, `kind`, `tool`, `moves`, `cut_length`, `cut_minutes`,
`max_tilt_deg`, `a_deg` when indexed, `gouge`, `holder_collisions`, `warnings`), `program_lines`,
`program_head` (the first lines), `readback`, `cut_minutes` (as the machine runs it with
`machine_rates`, else as written), `rapid_minutes` and `machine_time` (with `machine_rates`, else
`null`), `gouge`, `holder_collisions` (`null` when no tool has holder data), `rapids` (`checked`, `hits`, `worst`, `pass`, `from_unknown`, `first`),
`travel` (`checked`, `violation`, `pass`; `null` without `linear_limits`), `stock` (as checked
against; `null` without one), `passed`, `failures`, `dry_run`, `outputs` (`gcode`, `preview`,
`setup_sheet`), `notes` (how the bodies were tessellated, a post that writes no work offset, an
output extension other than the post's), `simulation` (with `simulate`, as `cam.simulate`
replies) and `summary`, the setup sheet's tables in compact form (see
[Setup sheet](#setup-sheet)), on a dry run too. An adaptive operation's entry in `operations`
also has `adaptive`, the engagement measured along its path.

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
| `stock` | The raw material in work coordinates: `{"box": {"min": [x, y, z], "max": [x, y, z]}}`, `{"cylinder": {"radius", "x_min", "x_max"}}` (a bar about work X, the A axis) or `{"around_part": {"margin": [x, y, z]}}` (the part's box grown by the margin each side). No rapid may pass through it; `face` faces it. Default: none. |
| `tools` | The tools, as the CAM crate's `Tool` JSON (below). Give this or `tool_library_path`. |
| `tool_library_path` | A JSON file holding an array of tools. |
| `operations` | 1 to 200 operations, run and written in order (below). |
| `machine_rates` | The machine's top rates (below, "Machine rates"): the reply then times the program as the machine runs it and warns about blocks its controller would slow. Default: none. |
| `machine_rates_path` | A JSON file holding `machine_rates`, read from the job file's folder when relative. |

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

`number`, `kind` (`flat_endmill`, `ball_endmill`, `bull_nose`, `drill`, `face_mill`, `chamfer`,
`thread_mill`), `diameter`, `flute_length`, and optional `description`, `flutes`,
`spindle_rpm`, `feed`, `plunge_feed`, `stepdown`, `stepover`, `corner_radius` (bull nose; a
face mill's inserts), `tip_diameter` and `angle_deg` (chamfer mill, below), `shank_diameter`,
`stickout`, `holder: [{diameter, length}]`, `point_angle_deg` (a drill's point, default 118),
and for a thread mill `thread_pitch` and `tooth_height` (see
[Helical bores and thread milling](#helical-bores-and-thread-milling)). An operation without
`spindle_rpm` or `feeds` takes the tool's; its
stepdown and stepover default to the tool's too.

### Which tool for which operation

Every operation refuses a tool that cannot do its cut, with the reason, before planning:

| Kind | Used by | Refused by |
| --- | --- | --- |
| `flat_endmill`, `bull_nose`, `ball_endmill` | every milling operation; a flat end mill may also plunge holes in `drill` | `drill` (a ball or bull nose), `chamfer`, `thread_mill` |
| `face_mill` | `face`, `contour`, `pocket`, `adaptive`, `bore` and the surface finishes, always entering along a ramp or helix | `contour` or `pocket` that would plunge (`ramp_angle_deg` 0 and no ramp or helix entry), `roughing`, `rotary_roughing`, `drill`, `chamfer`, `thread_mill` |
| `chamfer` | `chamfer`; the surface finishes (`parallel`, `waterline`, `scallop` and the like; on steep walls only the cone's top rim cuts) | `contour`, `pocket`, `adaptive`, `adaptive_roughing`, `bore`, `roughing`, `rotary_roughing`, a swarf cut |
| `drill` | `drill` | every milling operation |
| `thread_mill` | `thread_mill` | every other operation |

The gouge check and the simulation take each tool's real cutting end: a chamfer mill's flat tip
and cone, a face mill's inserts (square, or rounded by `corner_radius`), a drill's point at its
own `point_angle_deg`, and a thread mill's whole radius (in `thread_mill` the check takes its
core instead, below).

### Operations

Every operation has `name`, `type`, `tool` (a tool's `number`), and optionally `spindle_rpm`,
`feeds: {cut, plunge}`, `coolant` (default false) and `gouge_check` (default true). `heights`
are `{clearance, retract, top, bottom}` in the operation's coordinates.

| `type` | Keys (defaults) |
| --- | --- |
| `contour` | `sketch` (id or name) + `profiles?` (indices; default every drawn profile) or `polygon: [[x, y], …]`; `heights`; `stepdown` (tool's, else half its diameter); `side`: `outside` \| `inside` \| `on` (outside); `climb` (true); `stock_to_leave` (0); `ramp_angle_deg` (0: plunge); `tolerance` (the job's); `lead`, `entry`, `linking`, `tabs`, `finish_passes`, `finish_stepover`, `spring_pass` (see [Leads, entries, linking and tabs](#leads-entries-linking-and-tabs)) |
| `pocket` | `sketch` + `profiles?` or `polygon`; `heights`; `stepdown`; `stepover` (tool's, else 40% of its diameter); `climb`; `stock_to_leave`; `ramp_angle_deg`; `tolerance`; `entry` (a helix); `linking` (see [Leads, entries, linking and tabs](#leads-entries-linking-and-tabs)) |
| `parallel` | `heights` (`bottom` is the floor); `stepover` (tool's, else 10% of its diameter); `angle_deg` (0); `boundary?: {rect?: [[x, y], [x, y]], polygon?}` (the part's extent); `stock_to_leave` (0); `step` (a quarter of the diameter); `tolerance`; `zigzag` (true) |
| `roughing` | `heights` (`top` is the stock's top); `stepdown`; `stepover`; `angle_deg`; `boundary`; `stock_to_leave` (0.25); `step`; `tolerance`; `zigzag` |
| `waterline` | `heights` (levels from `top` to `bottom`); `stepdown` (tool's, else a quarter of its diameter); `boundary`; `stock_to_leave` (0); `step`; `tolerance`; `climb` (true) |
| `scallop` | `heights` (`bottom` is the floor); `scallop_height` or `stepover` (tool's, else 10% of its diameter; along the surface); `direction`: `inward` \| `outward` (inward); `boundary`; `stock_to_leave` (0); `step` (a tenth of the diameter: the grid and the samples); `tolerance`; `climb` (true). See "3D finishing strategies". |
| `steep_shallow` | As `scallop`, with `threshold_deg` (40), `overlap` (a tenth of the diameter), `stepdown` (the spacing × sin threshold), `shallow`: `scallop` \| `parallel` (scallop), `angle_deg` (with `parallel`; 0) |
| `pencil` | `heights`; `min_angle_deg` (30); `passes` (0: the crease alone); `stepover` (tool's, else 10% of its diameter); `min_length` (the tool's radius); `boundary`; `stock_to_leave` (0); `step`; `tolerance`; `climb` |
| `rest_finish` | `heights`; `previous_tool` (a tool number of the job); `threshold` (twice the tolerance); `scallop_height` or `stepover`; `overlap` (a quarter of the diameter); `boundary`; `stock_to_leave` (0); `step`; `tolerance`; `climb` |
| `drill` | `sketch` (circle centres, `circles` (true), filtered by `diameter_range: [min, max]`, and lone points, `sketch_points` (true)) or `points: [[x, y], …]`; `heights` (`retract` is the R height, `top` the hole's top, `bottom` its depth); `cycle`: `simple` \| `dwell` \| `peck` \| `chip_break` (simple); `peck` (tool's stepdown, else its diameter); `peck_clearance` (0.5); `back_off` (0.5); `dwell_seconds` (0.5); `to_full_diameter` (false); `stay_at_retract` (false). See [Drilling](#drilling). |
| `adaptive` | `sketch` + `profiles?` or `polygon`; `heights`; `stepdown` (tool's, else its diameter); `optimal_load` (tool's `stepover`, else 10% of its diameter); `min_engagement_deg`, `max_engagement_deg` (from the load: see [Adaptive clearing](#adaptive-clearing)); `helix_diameter` (80% of the tool's); `ramp_angle_deg` (2); `climb`; `stock_to_leave` (0); `tolerance`; `compare` (true) |
| `adaptive_roughing` | `heights` (`top` is the stock's top); `stepdown`; `optimal_load`; `min_engagement_deg`, `max_engagement_deg`; `helix_diameter`; `ramp_angle_deg`; `climb`; `boundary` (the stock's extent; the part's extent); `stock_to_leave` (0.25); `flats` (true); `tolerance` |
| `chamfer` | `sketch` + `profiles?` or `polygon`; `heights: {clearance, retract, top}` (no `bottom`: the depth follows from the width); `width` (across the top); `tip_offset` (0.2); `side`: `outside` \| `inside` (outside); `climb` (true); `tolerance`. A chamfer mill only; see [Chamfers](#chamfers) |
| `indexed` | `a_deg` and `operation`: any of the 3-axis operations above, run with A held at `a_deg` (3+1). Only these two keys; the inner operation carries the name, tool and feeds. |
| `rotary_finish` | `pattern`: `{type: "lines", angle_step_deg, zigzag?}`, `{type: "helix", pitch}` or `{type: "rings", stepover, zigzag?}` (below); `clearance` and `retract` (radii from the A axis); `floor` (0); `range: [min x, max x]` (the part's extent); `step`; `tolerance`; `stock_to_leave` (0); `max_angle_step_deg` (2) |
| `rotary_roughing` | As `rotary_finish`, with `stock_radius`, `stepdown` (in radius) and `stock_to_leave` (0.25) |
| `bore` | `sketch` (its circles) + `diameter_range?: [min, max]` or `holes: [{center: [x, y], diameter}]`; `heights` (`top` the hole's top, `bottom` its floor); `pitch` or `ramp_angle_deg` (2, the pitch at most `stepdown`: the tool's, else 10% of its diameter); `stepover` (between rings; the tool's, else 40% of its diameter); `climb` (true); `finish_passes` (0); `stock_to_leave` (0.1 with finishing passes, else 0) |
| `thread_mill` | `sketch` (its circles' centres) + `diameter_range?` or `centers: [[x, y], …]`; `heights` (the thread from `top` down to `bottom`); `pitch`; `thread_diameter` (the major, nominal diameter); `internal` (true); `right_hand` (true); `climb` (true); `thread_depth` (ISO metric: 0.5413 × pitch inside, 0.6134 × pitch outside); `radial_passes` (1); `spring_pass` (false) |
| `face` | `bottom`; `top` (the stock's top; required without a stock); `retract` (2 above the top, or the stock's top if higher); `clearance` (10 above the retract); `stepdown` (tool's, else 1); `stepover` (tool's, else 70% of the flat of its end); `angle_deg` (0); `zigzag` (true); `overhang` (half the tool's diameter and 0.5); `rect: [[x, y], [x, y]]` (the stock seen from above, else the part's extent). A face mill, a flat end mill or a bull nose. |
| `wrap` | `radius` (of the cylinder the path is drawn on); `points: [[x, y, z], …]` (x along the axis, y the arc length round it, z the depth: 0 on the cylinder, negative into it) or `sketch` + `curve` (a curve id; its sketch x and y are taken as x and arc length) with `depth` (0); `clearance` (5) and `retract` (1) above the cylinder; `max_angle_step_deg` (2) |
| `wrap_pocket` | `radius` (of the cylinder the profile is drawn on); `sketch` + `profiles?` or `polygon: [[x, arc length], …]` (unrolled: x along the axis, y the arc length round the cylinder); `bottom` (the floor's depth from the cylinder, negative into it); `top` (0); `stepdown`; `stepover`; `clearance` (5) and `retract` (1) above the cylinder; `climb`; `stock_to_leave` (walls); `ramp_angle_deg`; `tolerance`; `max_angle_step_deg` (2). See "Wrapped pockets and contours" below. |
| `wrap_contour` | As `wrap_pocket`, with `side` (outside) instead of `stepover`: the profile's boundary followed in depth levels |
| `indexed_pattern` | `a_deg: [angles]` or `count` (evenly over a turn, from `start_deg`, 0), and `operation`: any 3-axis operation, run as an `indexed` operation at each angle, named `<name> A<angle>` |

The 3-axis operations (`contour` to `chamfer` in the table) run with every rotary at zero; on `3axis` and
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

## Drilling

A `drill` operation feeds a drill (or a flat end mill, plunging) straight down the tool axis at
each hole, at the operation's plunge feed. Any other tool is refused. It is a 3-axis operation,
so it also runs inside `indexed`: its points and heights are then in the frame the angle brings
under the tool.

```json
{"name": "holes", "type": "drill", "tool": 3, "sketch": "Holes", "diameter_range": [4, 5],
 "heights": {"clearance": 15, "retract": 2, "top": 0, "bottom": -12}, "cycle": "peck", "peck": 4}
```

**Holes.** With `sketch`: the centres of its circles (not construction ones; with
`diameter_range`, only circles of a diameter in it) and its points that are not part of a curve.
The sketch must lie on a plane square to the tool axis (either way up). With `points`: the
holes in the operation's XY. Repeated positions are drilled once. Holes are drilled in a greedy
nearest-neighbour order from the one lowest in X (then Y), the same order every time.

**Cycles.** Every cycle is written as plain `G0`/`G1` moves and a `G4` dwell, which every
controller reads and the read-back follows; no canned cycle (`G81`, `G82`, `G83`, `G73`) is
written.

| `cycle` | Each hole |
| --- | --- |
| `simple` | Feed from the R height to the bottom, rapid out to R. |
| `dwell` | The same, with a pause of `dwell_seconds` at the bottom. |
| `peck` | Feed `peck` deeper at a time, counted from `top` (the last peck ends at the bottom); after each peck rapid right out to R, then rapid back down to `peck_clearance` above the depth reached. |
| `chip_break` | Feed `peck` deeper at a time; after each peck rapid back `back_off` (staying in the hole) to break the chip, then feed on. |

**Depth.** `bottom` is where the tip goes. With `to_full_diameter: true` it is where the drill
reaches its full diameter, and the tip goes deeper by the point's length, radius / tan(point
angle / 2): 1.80 mm for a 6 mm drill with a 118° point. A hole deeper than the tool's flutes is
refused.

**Between holes** the tool rises to the clearance height. `stay_at_retract: true` travels at the R
height instead, and is refused unless the R height is above the highest point of the part and the
fixtures in the operation's frame.

**Checks.** The gouge check holds a drill against the part as its real shape: the full diameter
from the point's length up, and the point as stepped cylinders inscribed in its cone (within the
job's tolerance along the axis, at most 64 steps). Model the holes as cut bodies; drilled into a
part that shows no hole, the operation gouges and the job is refused.

The tool comes down from the clearance height to the R height at rapid over every hole, and the
gouge check cannot see material there (the finished part already has the hole). So the operation
is refused when the part or a fixture within the tool's radius and a tenth more of a hole's axis
(the rim of a hole of the tool's size) is above the R height by more than the tolerance: a hole in
a boss drilled with the heights of the plate under it, say. Drill holes whose tops are at
different heights in separate operations, each with its own `top` and `retract`.

## Helical bores and thread milling

`bore` and `thread_mill` follow a helix round each hole's axis (+Z of work coordinates, every
rotary at zero) and are written with helical arcs (`G2`/`G3` with Z, half a turn each) where the
post has `helical_arcs`. Their holes are a sketch's circles (exact centres and diameters, on a
plane facing +Z; construction circles are left out), optionally only those whose diameter is in
`diameter_range`, or a list. Both are read back and gouge-checked like every operation, so the
model should show the holes (and, for a thread, at least its minor diameter).

```json
{"name": "bores", "type": "bore", "tool": 1, "sketch": "Holes", "diameter_range": [8, 20],
 "heights": {"clearance": 20, "retract": 5, "top": 0, "bottom": -12}, "ramp_angle_deg": 2, "finish_passes": 1}
```

A **bore** goes to each hole's centre, feeds down to `top`, and runs a half-circle lead-in
inside the hole onto its helix, so the tool reaches the wall along it and never head-on. The
helix goes down `pitch` per turn (or slopes at `ramp_angle_deg` at the tool centre, its pitch
then at most `stepdown`), climb milling counter-clockwise by default, and never goes down faster
than the plunge feed: a steep helix close round the axis, nearly a plunge, is slowed to keep its
Z at the plunge feed. A full flat circle cleans
the floor, a half-circle lead-out goes back to the centre, and the tool rises inside the hole.
A hole more than two tool diameters across is opened in several helical rings, the first within
the tool's radius of the centre (so no core is left) and each further one at most `stepover`
out. With `finish_passes`, roughing leaves `stock_to_leave` on the wall and the finishing passes
go round at the full diameter (after the first they are spring passes). A hole that is not at
least 0.1 mm wider than the tool is refused, the hole named (`hole 2 at (10, 0), ⌀5 mm`); one
narrower than 1.2 tool diameters is cut with a warning, as it leaves little room for chips.

A **thread mill** is a tool of `kind: "thread_mill"`: `diameter` across the tips of its teeth,
`thread_pitch` for a multi-form tool (rows of teeth that far apart over its `flute_length`;
left out, a single-point tool that cuts any pitch) and optionally `tooth_height` (how far its
teeth stand out of its core). Z is the tool's tip; a single-point tool's tooth is taken there.
A **thread_mill** operation cuts `thread_diameter` (the major diameter) at exactly `pitch` per
turn: inside a hole (`internal`, the default) the tool reaches the major diameter, round a boss
the minor one. The depth is split into `radial_passes` from the minor (inside) or major (outside)
diameter, then `spring_pass` repeats the last. Climb milling goes counter-clockwise inside a hole
and clockwise round a boss, so a right-hand thread is cut from the bottom up inside and from the
top down outside; `right_hand: false` or `climb: false` turns that round. Inside, each pass
starts at the hole's centre with a half-circle lead-in and ends with a lead-out back to it.
These leads turn the tool a quarter turn about the axis, so they rise (or fall) a quarter pitch
with the helix and the teeth stay on the thread: the tip runs from `bottom` to `top`, the full
depth from a quarter pitch above the one to a quarter pitch below the other, and an internal
thread must be at least half a pitch long. Outside, the lead-in comes from just clear of the
thread and the leads stay level (they turn the tool only a few degrees about the axis). A
multi-form tool makes one turn per span of its teeth from the bottom up; its last turn starts as
low as still reaches the top, a whole number of pitches above the others, so its teeth cut at
most a pitch of thread beyond the top (a tool whose teeth are longer than the thread is warned
of). A thread mill that does not fit the minor diameter is refused, and so is a multi-form tool
of another pitch.

The gouge check cannot see a thread the model does not show, so for a thread_mill operation it
takes the thread mill as a flat end mill at the roots of its teeth, the thread's depth inside its
tip radius, under the thread mill's own shank and holder: that core must stay clear of the part,
while the teeth beyond it cut the thread. The program still names the thread mill. Thread mills
are refused by every other milling operation, and anywhere else a thread mill is checked whole,
its teeth and all.

## Stock, facing and rapid safety

A job may say what it starts from with `stock`, in work coordinates. A box gives its corners;
`around_part` is the box round the part's bodies grown by a margin on each side (the reply's
`stock` shows the box it came to); a cylinder is a round bar about work X, the A axis of a 4th
axis set up the usual way. The notes say when the part reaches outside the stock.

### Facing

`face` cuts the stock's top (or `top`) flat down to `bottom` in levels of at most `stepdown`,
in straight passes along `angle_deg` at most `stepover` apart, over `rect` (by default the
stock seen from above). Every pass starts and ends with the whole cutter 1 mm clear of the
area, so the tool goes down beside the stock at the plunge feed and enters it at depth: it
never plunges into material. The outermost passes reach `overhang` past the edges (by default
half the diameter and 0.5 mm, which puts their centres just outside), so the edges come out
clean. With `zigzag` every other pass runs back and the passes link at depth along a line
beside the stock, each level starting where the last ended; without it the tool rises to the
retract height between passes and every pass runs the same way. The stepover is held to the
flat of the tool's end (its diameter less the corners, for a bull nose), so the floor has no
ridges. A top below the stock's top is warned of: its first level takes more than a stepdown.
With a stock, a `face` is refused where a descent or a link between passes would run in the
stock (a `rect` that stops short of the stock's edges along the passes would take the tool
straight down into it: give one that reaches past them, or turn the passes), and where its
`bottom` is below the stock's (whatever holds the stock is there).

### The rapid check

Every `G0` block of the program as written is read back and followed as the controller moves
through it, every axis evenly from the block's start to its end, sampled every 0.5 mm of the
tip's travel and of a point at the tool's reach. That covers the move to the first point, the
links between passes and operations, rotary turns, rewinds, the moves after a tool change and
the final retract. At every sample, in work coordinates:

- the cutter, as a cylinder of its radius without end above its tip (the shank and spindle are
  there too), must not be in the part or a fixture by more than the job's tolerance, and a
  tool's shank and holder (with holder data) must not touch them. In an operation with
  `gouge_check: false` (it cuts what the model does not show, a deburr of a sharp-edged
  model), a retract straight up the tool's axis is not held against the part: it leaves the
  cut its own feed moves made;
- with a stock, the cutter must not be in the stock. Nothing tracks what earlier operations
  removed, so the stock as given is the bound: a rapid below the stock's top over the stock
  fails, even where an earlier operation has cut it away, except after a `face` (with its
  default `overhang`) over the whole top of a box stock: the operations after it are held
  against the stock with its top at the face's `bottom`. A retract straight up the tool's
  axis out of the cut is exempt too: it only passes through room the tool already fills; so is
  a rapid back down that line in the same operation, no deeper than the retract started (a peck
  drill going back into its hole to `peck_clearance` above the depth reached). Keep
  every retract and clearance height above the stock's top (on a 4th axis, outside its radius);
- a block that turns the A axis (a table about X) must keep the cutter outside the radius the
  stock sweeps (a box's farthest corner) all through the turn, wherever the stock is level with
  the tool along X. Without a stock the part is cut into 128 slices along X, each held to the
  largest radius of the part over it and the tolerance, so a turn over a slender end of the
  part is held to that end's radius, as the 4th-axis setup check holds it.
  A post with `clearance_retract` rises to the operation's highest rapid before it turns; a
  low clearance height can still leave the tool inside the swept radius, and this check finds it.

A block after a return to the machine's home (`G28`, `G30`), a move in machine coordinates
(`G53`) or a tool change starts where the program does not say: only its end is checked, which
with the cutter endless above covers the way down from the machine's top. Turns made there (a
post's `safe_rotate` lines) are taken to happen at the machine's top, and a tool change to be
made where the new tool clears everything: give the post a `safe_retract` at tool changes where
the controller changes tools wherever the spindle is. `rapids.from_unknown` counts these blocks.
Each block is checked with the tool of its own operation.

A failure names the line, the operation, what the rapid runs into, how deep and where; the
reply's `rapids.first` lists the first 20. Like the other checks, a failure refuses the call
unless `dry_run` or `allow_unsafe` is given.

### Travel

A machine given as JSON with `linear_limits: [[x, y, z], [x, y, z]]` (the lowest and highest
position of the linear axes, as written in the program) has every block's end, and points along
every arc, held against them. The first position beyond them fails the job, by line and axis.

## Adaptive clearing

`adaptive` (2D, like `pocket`) and `adaptive_roughing` (3D, like `roughing`) clear material with
the tool's engagement held down: the share of its circumference in material never goes above
`max_engagement_deg` outside an entry helix, so the tool never runs in a full slot. This suits a
small, not very rigid mill best: take a deep `stepdown` (the flute length the tool's data allows)
with a light `optimal_load` (5% to 15% of the diameter) and the full feed.

The path is planned on a raster of the material a sixteenth of the tool radius fine, with exact
distances to the path already planned near the edge of the cut. It is walked in short steps,
turning at most 25° a step, so its corners are rounded; at each step it takes the heading whose
engagement is nearest the optimal load's (on a straight edge `acos(1 − load / radius)`), with the
material on the climb (or conventional) side, and never one above the limit. Where a walk can
cut no more, the next starts where cleared space meets material, reached by a feed through
cleared material when there is one with the tool down, by lifting to `retract` otherwise.

- **Entry.** Where the tool can reach the material from open air (round the stock in
  `adaptive_roughing`) it goes down outside it. Material enclosed on every side is entered with
  a helix of `helix_diameter` at `ramp_angle_deg`, then a circle at the level; with no room for
  the helix the operation is refused, saying so.
- **Levels.** `adaptive` cuts the same path at every depth pass. `adaptive_roughing` plans each
  level where the dropped cutter (grown by `stock_to_leave`) is at or below it; with `flats` it
  adds a level at every flat area of the part (raised by the stock to leave). Levels further
  apart than the tool's `flute_length` are refused: the shank would rub on the wall the flutes
  leave.
- **Corners.** An inside corner sharper than the limit allows is not driven into: a fillet
  somewhat larger than the tool's is left there for a finishing contour (`left_area` in the
  report).

Defaults: `optimal_load` is the tool's `stepover` (a tool library's adaptive start values), else
10% of its diameter; `max_engagement_deg` one and a half times the optimal load's engagement and
at least 15° more than it; `min_engagement_deg` a third of it, and at most half the
`max_engagement_deg`. A `max_engagement_deg` below the load's engagement wins: the path aims at
the narrower width the limit allows, which is slower to plan; a width under about 3% of the
tool's radius can be refused as too much work.

- **Stock.** `adaptive_roughing` takes the material as the box over `boundary` (its rectangle,
  and inside its polygon too) from `top` down, and goes down to a level in the open air outside
  it. Without `boundary.rect` the stock is the part's extent seen from above, and the report
  warns: a blank larger than that would be cut into from the side at the level's full depth, so
  give the blank's extent as `boundary.rect`. Nothing round the stock is modelled: keep the
  levels that go down outside it above the vise jaws and clamps.

Each adaptive operation's report carries `adaptive`, measured along the path as written,
independently of the planner (exact containment in the profiles or stock, exact distances to
every earlier move, the tool's circumference looked at in 1° steps): `measured` (false when the
path was too long to measure: the figures are then 0, and a warning says so), `max_engagement_deg`,
`max_entry_engagement_deg`, `mean_engagement_deg`, `full_slot_share` (of the cutting time
outside entries at 170° or more: 0), `entry_slot_share`, `limit_deg`, `levels`, `helices`,
`lifts`, `left_area` and, for `adaptive` with `compare`, `pocket`: a plain offset pocket of the
same profiles (stepover 40% of the diameter) measured the same way (`max_engagement_deg`,
`full_slot_share`, `cut_length`, `cut_minutes`). A measured engagement more than 3° above
`limit_deg`, or any full slot outside the entries, is also a warning of the operation. Both
operations also work inside `indexed`.

## Chamfers

A face mill (`kind: "face_mill"`) is a flat end, its inserts' corners rounded by
`corner_radius` when given; its tool-list name is `FACE MILL`. A chamfer mill (`kind:
"chamfer"`) has `diameter` (the widest the cone cuts), `tip_diameter` (0 for a pointed tip,
default 0) and `angle_deg`, the included angle (90 for 45° chamfers, 1 to 179, required); its
tool-list name is `CHAMFER MILL`. Contours and pockets refuse a chamfer mill. The drop-cutter
and the gouge check know its true shape, a flat tip and a cone, so the finishing operations
(`parallel`, `waterline`, `rotary_finish`, `wrap`, 5-axis surface finishing) can use one.
`roughing` and `rotary_roughing` refuse it, since its point leaves ridges between passes that
the roughing would take as cut, and so does a swarf cut, since its side is tapered.

A face mill's inserts do not cut at its centre, so it never goes straight down into material:
a `contour` or `pocket` with a face mill needs `ramp_angle_deg` above 0, and `roughing` and
`rotary_roughing`, which plunge, refuse it.

The `chamfer` operation cuts a chamfer along the top edges of closed profiles in one pass:

```json
{"name": "deburr", "type": "chamfer", "tool": 7, "sketch": "Outline",
 "heights": {"clearance": 10, "retract": 2, "top": 0}, "width": 0.5, "side": "outside"}
```

`width` is the chamfer's leg across the top face; its leg down the wall is `width / tan(angle_deg
/ 2)` (the same for a 90° tool). The tip runs `tip_offset` below the chamfer so the cone's side,
not its tip, cuts all of it: `width / tan(angle_deg / 2) + tip_offset` below `top`, its axis
`tip_diameter / 2 + tip_offset · tan(angle_deg / 2)` out from the edge. A chamfer the cone cannot
reach, wider than `diameter / 2 − tip_diameter / 2 − tip_offset · tan(angle_deg / 2)`, or deeper
than the flutes, is refused with the widest it can cut, never cut narrower. `outside` chamfers a
part's outline, `inside` an opening's; corners of an outside chamfer are rounded by the cone.

The gouge check holds the cone to the model: chamfer edges the model has chamfered (with
`solid.chamfer` at the tool's angle) and it passes; a chamfer the model does not show cuts into
it, so a deburr of a sharp-edged model sets `gouge_check: false`. Inside `indexed` a chamfer runs
in the frame the A angle brings under the tool.

## Leads, entries, linking and tabs

The 2D operations (`contour` and `pocket`, on their own or inside `indexed`) take these keys.
Without them a contour cuts as before; a pocket goes in by a helix.

```json
{"name": "outline", "type": "contour", "tool": 1, "polygon": [[-20, -15], [20, -15], [20, 15], [-20, 15]],
 "heights": {"clearance": 20, "retract": 5, "top": 0, "bottom": -6}, "stepdown": 2,
 "lead": {"radius": 2, "line": 1}, "entry": {"type": "ramp", "angle_deg": 3},
 "tabs": {"count": 4, "width": 4, "height": 1.5}, "finish_passes": 1, "finish_stepover": 0.25,
 "spring_pass": true, "linking": "stay_down"}
```

| Key | Operations | Meaning |
| --- | --- | --- |
| `lead` | contour | `{radius, sweep_deg?, line?}`: a tangent arc into each pass and out of it, `sweep_deg` (90) round, with a straight `line` (0) before the lead-in and after the lead-out. It lies in the plane of the cut (`G2`/`G3`), on the side of the pass away from the part, and the pass starts in the middle of its longest straight piece. A lead that would cut into the part (the far wall of an opening, a second profile) is refused, and so is any lead with `side: on`, where both sides of the line are the part. |
| `entry` | both | How the tool goes down into each depth pass: `{"type": "plunge"}` (at the plunge feed), `{"type": "ramp", "angle_deg"?}` (2°: a contour along its lead, back and forth, or before its first tab, else round the loop; a pocket round its ring) or `{"type": "helix", "radius"?, "angle_deg"?}` (90% of the tool's radius, at most all of it, so that no core is left standing; 2°), then once round at the depth. A helix that does not fit is made smaller, then replaced by the ramp, then by the plunge; a contour with `side: on` takes no helix. Give `entry` or `ramp_angle_deg`, not both. Default: a contour plunges (or ramps at `ramp_angle_deg`); a pocket goes in by the helix. |
| `linking` | both | `clearance` (the default) or `stay_down`. Within a level the tool always stays down where the straight link runs where the passes cut so far, or the one it leads to, have taken the material away (tracked by each pass's distance from the boundary); elsewhere it goes up to the clearance height, or with `stay_down` only to the retract height where the cutter, dropped onto the part all along the link, stays 1 mm below it. With `stay_down` a pocket also goes from one level to the next through the level it has cleared. |
| `tabs` | contour, outside | `{count \| spacing, width, height, ramp_angle_deg?}`: material left `height` above the bottom to hold a part cut out of the stock, each `width` long along the wall, placed evenly on the profile's straight stretches (away from its corners and the start); the tool rises over each on ramps of `ramp_angle_deg` (30°). Refused when they do not fit or are as high as the cut. |
| `finish_passes` | contour | Passes round the final wall at full depth after the depth passes (0), `finish_stepover` (5% of the tool's diameter, at most half its radius) apart, the last on the wall; the depth passes leave `finish_passes × finish_stepover` for them. Each is linked in the cut, or by its lead. Not with `side: on`. Refused where a finishing pass would reach into a notch, a neck or a gap between parts that the pass before it did not fit in (it would cut a full slot at full depth). |
| `spring_pass` | contour | Go round the last pass once more (false). |

No cut of a contour or a pocket goes down faster than the plunge feed: a ramp, a helix and the
way down off a tab are slowed to the feed that takes the tool down at the plunge feed (a 30° tab
ramp at 2880 mm/min would otherwise go down at 1440 mm/min).

Ring by ring a pocket is cut from the inside out: a ring only after the rings next inside it, and
otherwise the nearest ring first, so that the rooms of a pocket are cleared one after the other.
The vertical leads some posts write (an arc in the plane of the cut's normal and Z, `G18`/`G19`)
are not written: the program reader and the read-back follow arcs in the XY plane only.

## 3D finishing strategies

`scallop`, `steep_shallow`, `pencil` and `rest_finish` place their passes by the shape of the
surface as the cutter meets it, where `parallel` and `waterline` use straight lines and flat
levels. All four start from a grid of drop-cutter samples `step` apart over the boundary (the
tip's height where the cutter rests, never below the floor `heights.bottom`) and measure
distances over that surface, so neighbouring passes are a spacing apart along the surface,
whatever its slope. Every pass is then sampled and refined with the drop-cutter exactly as a
`parallel` pass is: no point it cuts at is below where the cutter rests, moves that go down are
fed no faster down than the plunge feed, and passes are linked by feeding across when the link
is short and clears the part, otherwise by lifting to the retract height. They can run inside
`indexed` like the other 3-axis operations. With `climb` (the default) every offset pass and
z-level keeps what is still to cut on the right of the tool's travel (the spindle turning
clockwise seen from above), whichever way the passes run: inward, outward, or closing on a
crease; `climb: false` keeps it on the left. A grid larger than 4 million nodes, more than
10 000 levels, straight shallow passes of more than 4 million points or more drop-cutter
samples than the operation's budget is refused with a message saying what to make larger.

- **`scallop`** (constant stepover): passes offset from the boundary inward (`direction:
  "outward"` runs the same passes from the middle out). With `scallop_height` the spacing is
  `2·√(2·r·h − h²)` for the cutter's corner radius `r` (a flat end mill needs `stepover`),
  closer where the surface is concave and further where it is convex, so the ridge left is the
  same. Where fronts from different sides meet further apart than the spacing, a pass is added
  between them. A last pass runs along every concave crease sharper than 30° (the foot of a
  wall, a corner), which offsets land a grid step short of at best. Walls the cutter's side runs
  down (cliffs in the height field) are not followed: use `steep_shallow` there.
- **`steep_shallow`**: where the tip's height is steeper than `threshold_deg`, z-levels at most
  `stepdown` apart, each crossing found to within the tolerance; elsewhere `scallop` passes
  offset from the boundary and from the edge of the steep ground, or straight passes at
  `angle_deg` (`shallow: "parallel"`), close enough to be a spacing apart on the steepest
  shallow ground. Each kind runs `overlap` on into the other's ground; the crease passes of
  `scallop` finish it.
- **`pencil`**: along the concave creases where the cutter touches two faces at once. A crease
  is where the tip's path turns up by at least `min_angle_deg` at one place across it, more than
  it turns either side (so a fillet the cutter rolls along is no crease); each crease point is
  moved across the crease to where the tip's height leaves the line of its flatter side, found by
  halving, so the pass runs in the crease, not on the nearest grid node. `passes` adds offset
  passes `stepover` apart along the surface on both sides, outermost first.
- **`rest_finish`**: only where this tool reaches material `previous_tool` left. At each grid
  node the tool's contact point and the surface's normal there come from its tip and the slope
  of the tip's height; the previous tool is set touching the same point with the same normal and
  dropped. Where it rests higher than that, what it left at the point is measured from the
  envelope of every position it can take on the grid (the lowest its cutting surface reaches
  over the point), square to the surface. Places with more than `threshold` left, grown by
  `overlap`, are cut with passes offset from their edges inward, and their creases.

Measured on test parts with a 6 mm ball and `scallop_height: 0.01` (the material left over what
any path with the tool could reach, from the envelope of the ball along the path, computed
independently of the strategies on a 0.1 mm grid; `crates/cam/src/finish3d_tests.rs`):

| Part | Strategy | Largest | 95% of points under | Points within 0.0125 mm |
| --- | --- | --- | --- | --- |
| Dome (radius 12, 7 mm high) | `scallop` | 0.019 mm | 0.0096 mm | 99.7% |
| Bowl (60° wall, concave fillet radius 4) | `scallop` | 0.021 mm | 0.0109 mm | 96.6% |
| Bowl | `steep_shallow` (40°, scallop) | 0.023 mm | 0.0100 mm | 97.4% |
| Bowl | `steep_shallow` (40°, parallel) | 0.027 mm | 0.0076 mm | 98.6% |
| Three treads 4 mm apart | `scallop` | 0.018 mm | 0.0090 mm | 99.95% |
| Three treads | `steep_shallow` (45°) | 0.016 mm | 0.0094 mm | 99.9% |

The largest values are at single points where offset fronts meet. On the dome a plain
`parallel` raster at the stepover the scallop height gives on flat ground does about as well
(0.019 mm, 95% under 0.0087 mm, 98.5% within) with a shorter path (1860 mm against 2470 mm): the
dome shows no advantage for `scallop`, and the other parts were not compared with a raster. On a
sharp wall-and-floor corner a 10 mm ball leaves up to 5.6 mm; `rest_finish` with a 4 mm ball after it leaves at most
0.016 mm, and cuts nothing on the floor further out or on the top. A pencil pass on a fillet
joint tighter than the ball runs within 0.003 mm of where the ball touches both faces.

## Wrapped pockets and contours, rings and indexed patterns

More 4th-axis work for a table about +X (`machine: "4axis_a"`), all checked like the others:
the program is read back and every cut is checked for gouges.

### Wrapped pockets and contours

`wrap_pocket` and `wrap_contour` cut a closed 2D profile into a cylinder of `radius` round the
A axis, the tool pointing at the axis. The profile is drawn unrolled: x along the axis, y the
arc length round the cylinder at `radius` (a polygon, or a sketch on a plane facing +Z whose
x and y are taken that way, in work coordinates). It is cut in levels from `top` down to
`bottom` (depths from the cylinder, negative into it), at most `stepdown` apart; the floor is
the cylinder of radius `radius + bottom`, exactly.

Unrolling distorts with depth. The profile keeps its angles as it goes down (a wall at arc
length y is the radial plane at A = y / radius), but a tool of radius r at a level of radius ρ
must have its centre r from the wall in arc length *at ρ*, which is an angle of r / ρ: wider in
angle the deeper the level. Offsetting once at the cylinder and wrapping that at every depth
would cut each wall too far round by r·(1/ρ − 1/radius) of angle at the floor. So every level is
planned on its own: the profile's arc lengths are scaled by ρ / radius (the unrolled plane at the
cutting radius, where lengths are true), the pocket or contour is offset there and wrapped at ρ.
Along X nothing is distorted. A radial tool's side is parallel to its axis, so above the tip it
stays farther from a radial wall than at the tip: a wall is left as steps no taller than a
stepdown, and at the tip at most r − ρ·atan(r / ρ) ≈ r³ / (3ρ²) short of it (0.02 mm for a 6 mm
tool at radius 20), never cut into.

Lines are kept exact; curves and the offset arcs are followed in chords within a quarter of the
tolerance each and the wrapped moves within half of it. Cuts are written as short moves, each
turning A by at most `max_angle_step_deg`, in inverse time where the post says so; A stays
within the profile's angles, and a post with `rewind` brings a long cut back within the table's
limits. Everything one level cuts must fit within one turn at that level, or the operation is
refused. Text engraving is not part of these operations.

### Rings

`{"type": "rings", "stepover": 2, "zigzag": true}` is a third `pattern` for `rotary_finish` and
`rotary_roughing`: circumferential passes, a full turn of A at each X, X stepping between them.
The rings are evenly spaced over `range`, both ends included, at most `stepover` apart (exactly
it when it divides the range). With `zigzag` (the default) every other ring turns back, so A
stays within one turn; without it every ring turns the same way, one cutting direction
throughout, and A winds on a turn per ring (the post's `rewind`, or a table that turns without
end). Rings are linked by feeding across X when that is short and clear. For roughing, rings
farther apart than the tool is wide are refused.

### Indexed patterns

```json
{"type": "indexed_pattern", "a_deg": [0, 90, 180, 270],
 "operation": {"name": "flat", "type": "parallel", "tool": 1, "heights": {"clearance": 40, "retract": 17, "top": 15, "bottom": 0}}}
```

runs one 3-axis operation at several A angles, as that many `indexed` entries would: each is
named `<name> A<angle>` (`flat A0`, `flat A90`, …) and written exactly as an `indexed` entry with
that name and angle. `count: 6` spreads six angles evenly over a turn from `start_deg` (0). A
pattern has 1 to 200 angles, and a job's operations, every angle counted, are at most 200.

### The setup check

Before anything is written, every rotary operation's clearance is held against the part along
the stretch of X it works over (its moves, the tool's radius either side): where its rapids go
round and where A turns, the tool must be outside the part's largest radius about the A axis
there, by more than the job's tolerance. That is the `clearance` of `rotary_finish` and
`rotary_roughing` (which must also clear its `stock_radius`), and `radius + clearance` for
`wrap`, `wrap_pocket` and `wrap_contour`.

A turns into an operation where the one before it left the tool: the post rises there (to its
`clearance`, or else to the new operation's highest rapid, never down) and turns A before it
moves across. So whenever A turns between two operations with the same tool, of any kind (a
plain 3-axis one included), the tool's end at that height must be outside the part's largest
radius within the tool's radius of the X where it stands. Raising the earlier operation's
clearance, or the post's, fixes it. An `indexed` or `indexed_pattern` operation turns A only as
it starts; where the tool's place is not known (the program's first operation, or after a tool
change) its own clearance is held against the X it works over instead. A post whose
`safe_rotate` lines take the tool to the machine's top before every turn is not held to either.
A job that fails is refused with the operation, the height and how far the part (or the stock)
reaches.

## Machine rates

A controller moves no axis faster than its top rate. A rapid takes as long as its slowest axis
needs, and a cut written faster than an axis can go is slowed down, so it takes longer than
its `F` says. That includes a feed above the machine's top feed, and a cut that turns A
faster than A can turn, in inverse time (`G93`) or not. Give the job the machine's rates and
`cam.job` times the program as the machine runs it:

```json
"machine_rates": {"rapid_xyz_mm_min": 8000, "rapid_z_mm_min": 4000, "rapid_a_deg_min": 8000,
                  "max_feed_mm_min": 8000, "max_feed_z_mm_min": 4000, "max_a_deg_min": 8000,
                  "spindle_rpm": [10000, 24000], "spindle_start_seconds": 6, "travel_mm": [300, 170, 130]}
```

| Key | Meaning |
| --- | --- |
| `rapid_xyz_mm_min` | Rapid rate of X and Y, and of Z unless `rapid_z_mm_min` is given. Required. |
| `rapid_z_mm_min` | Rapid rate of Z, where it differs. |
| `rapid_a_deg_min` | Rapid rate of the rotaries, degrees a minute. Without it a turn at rapid is not timed (the reply says so). |
| `max_feed_mm_min`, `max_feed_z_mm_min` | Top feed of X and Y, and of Z. |
| `max_a_deg_min` | Top rate of the rotaries while cutting, degrees a minute. |
| `spindle_rpm` | `[lowest, highest]`: the controller runs any other `S` at the nearest end of its range. |
| `spindle_start_seconds` | How long the controller waits at each spindle start (`M3`, `M4`). |
| `travel_mm` | `[x, y, z]`: the travel of the linear axes. |

A rate left out is not limited. Every rate must be above 0. Unknown keys are refused, in a
`machine_rates_path` file too.

With rates, the reply's `cut_minutes` is the time at feed as the machine runs it (dwells
included), `rapid_minutes` the time at rapid, and `machine_time` has the detail:
`written_cut_minutes` (the `F` words alone), `spindle_minutes`, `total_minutes`,
`slowed_blocks` (and of those `rotary_over` and `feed_over`), `peak_rotary_deg_min` and
`peak_feed_mm_min` with their lines, `spindle_out_of_range`, `span_mm` and `warnings`. The
warnings are in the reply's `notes` too: blocks that ask A for more than `max_a_deg_min`
(the controller slows them, so the time is longer and the feed at the tool is lower than
written), blocks that ask a linear axis for more than its top feed, spindle speeds outside
`spindle_rpm`, and a program that spans more than `travel_mm`. Without rates, `cut_minutes`
is the time as written and `rapid_minutes` is `null`.

The times leave out acceleration, so they are a lower bound for programs of many short blocks.
Moves to and from the machine's home (`G28`) are in machine coordinates and are not timed.

### Controller settings a post must match

- **Dwells.** `G4 P` is read in seconds by some controllers and in milliseconds by others; a
  Mach3 profile has a setting for it. A post's `dwell` template gives the length as
  `{seconds}` or `{milliseconds}`, exactly one of them (a template with neither or both is
  refused). The read-back and the machine time read `P` in the same unit, so a post set to
  milliseconds is checked the same way.
- **Coolant and the index.** By default an operation switches its coolant on after its start
  lines, before its first turn of A. A post with `"coolant_after_turn": true` switches it on
  right after that turn (`G0 A90.` then `M8`). Coolant that goes off, and an operation that
  turns nothing at its start, are written the same either way.

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
`failures`, `warnings`, `notes` and `stock_stl`. A program with no move the simulation can
follow (on a 4th axis, one that never writes A: no block's position is known) fails rather than
passing with nothing checked. A failed simulation is reported, never refused;
inside `cam.job` its failures join the job's. When an operation of the job has `gouge_check:
false` (an engraving), the simulation's gouges are warned about, not failed.

**The stock.** The simulation starts from the job's `stock` when it can model it on the
machine: a box on a 3-axis machine, a bar about X on a 4th axis (`stock.derived` is then false,
and a face of the stock's top shows as removed volume). A job without a stock, or with one of
the other shape (a note says so), starts from one taken from the part: on a 3-axis machine the
part's box, on a 4th axis the cylinder about work X through the part's farthest point from the
axis, over the part's length. A program whose rapids are safe for a block-shaped part may
then cut that round bar at its retract height, which is what the bar would do on the machine.

**3-axis.** The stock is a heightfield over the box: one top height per cell, at the cell's
centre. Every block lowers each cell to the tool's lowest point over it along the whole move,
found exactly, not at sampled positions: along a straight move the tool's height over a cell is
convex in the position along the move, so its least value is found in closed form (flat end
mill, ball, drill point at 118°) or by bisection on its slope to rounding (bull nose). A level arc
is exact as well; an arc that also moves in Z (a helix) is followed as chords at most a quarter
of the tolerance inside it. An arc whose end is written at its start is a whole turn, as the
controller runs it. An arc in the XZ plane (`G18`, centre from `I K`, `G2` clockwise seen from
+Y) or the YZ plane (`G19`, centre from `J K`, `G2` clockwise seen from +X), as some posts write
lead-ins, is followed about its own centre, as chords at most a quarter of the tolerance inside
it. The tool is its cutting end with its diameter carried on up, so a cut
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

## Setup sheet

`cam.job {…, "setup_sheet": "C:/parts/bracket-setup.html"}` writes what the machine operator
needs to set the job up beside its program: a self-contained HTML page (no scripts, nothing
loaded from the network, print styles so it prints on one or two pages) for a `.html` or `.htm`
path, Markdown for `.md` or `.markdown`. Any other extension is refused before anything is
computed. Like the G-code and the preview it is written through the session's file access, so
an MCP server's `--allow-path` folders apply, and it must be a file of its own. The sheet carries
no date and nothing of the computer it is made on: the same job writes the same bytes.

It holds, in order:

- **The job:** its name, the machine, the post, the G-code file's name and the work offset it
  runs in (`G54` to `G59`, or a note that the post writes none).
- **Work zero:** the design point that is X0 Y0 Z0 and the part's extent from it, with each axis
  in words (on the part's left side, in the middle, on its top, so many mm below its bottom). On
  a 4th axis set up the usual way (`4axis_a`) Y0 Z0 is the A axis centreline, and the sheet says
  how far the part reaches from it.
- **Each rotary axis:** what it turns and about which line, which way positive turns it
  (counter-clockwise seen from the +X end for `4axis_a`; clockwise with `a_reversed`), which side
  of the part A+90° brings up under the tool and which way the top of the part moves as A
  rises, and its travel limits when the machine has them.
- **Stock**, when the job gives one: a block by its size and its corners from work zero (and
  the height of its top), a bar by its diameter and its ends along X.
- **Tools:** only those the operations use, by number: kind, diameter (and corner radius),
  flutes, flute length, shank, stickout, holder, the lowest Z the program sends the tip to
  (Z min, in program coordinates: on a 4th axis that is the distance from the A axis), the
  operations that use it, and its description. HTML sheets have a box to tick per tool.
- **Operations** in program order: name, kind, tool, spindle speed, cut and plunge feed,
  coolant, the A angle (`A90°`) or range (`A -180° to 180°`) as written, Z min, cut length and
  minutes at feed. Z min and the A angles are read from the written program itself, each
  operation's blocks being those after the comment the post writes at its start: an indexed
  operation shows the angle it asked for (`A-90°`, as written, not the equal `A270°`), and a
  cut on a table with limits shows the range the program turns through, rewinds included. With
  a post that writes no comments they are worked out from the toolpaths instead (indexed angles
  still as asked; a rewind's turn back is then not in the range).
- **Totals:** operations, tools, tool changes, length and time at feed as planned and as the
  controller times the written program, the program's line count, and the X, Y, Z and A range
  the program moves over.
- **Checks:** the read-back, the gouge check (with the operations not checked) and, when a tool
  has holder data, the shank and holder check, each with its result. The rapid, travel and
  simulation results are in the reply and its `failures`.
- **Warnings:** failed checks (a sheet is only written with them under `allow_unsafe`),
  operations not checked for gouges, tools without holder data, a rotary that turns past a full
  turn either way.
- **Notes:** how the bodies were tessellated and the like.

Every name and description from the job is escaped for the format, so a name such as
`<b>|*x*` shows as written and never breaks the page or a table.

The reply's `summary` carries the same tables in compact form, on a dry run too, so an agent can
tell the user what to set up without opening the sheet: `work_zero` (the sentences),
`part_extent` (`[[x, y, z] lowest, [x, y, z] highest]` from work zero), `stock` (the
sentence, `null` without one), `work_offset`, `rotary`
(the sentences), `tools` (`number`, `kind`, `diameter`, `flute_length`, `stickout?`, `holder?`,
`z_min?`, `operations`), `operations` (`name`, `kind`, `tool`, `spindle_rpm`, `feed`,
`plunge_feed`, `rotary?`, `cut_length`, `cut_minutes`), `totals` (`operations`, `tools`,
`tool_changes`, `cut_length`, `cut_minutes`, `program_minutes?`), `checks` and `warnings`.
