# CAM jobs

A CAM job turns a design's bodies into G-code for one machine. It is a JSON file kept next to
the design (or an object passed inline); nothing of it is saved in the `.sc` file. Three commands
work with jobs, from scripts, the CLI and MCP alike:

| Command | What it does |
| --- | --- |
| `cam.job` | Plans every operation, writes the program through the job's post, reads it back and checks it, and writes the G-code (and an HTML preview) |
| `cam.posts` | Lists the built-in posts, or returns one as JSON to copy into a post file |
| `cam.check` | Reads a G-code file and reports lengths, time, the rotary range, inverse-time blocks and the first line it cannot read |

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

Without `dry_run` or `allow_unsafe`, a failure refuses the call with every failed check in the
message. With `dry_run`, the reply carries `passed: false` and `failures`.

### The reply

`operations` (each with `name`, `kind`, `tool`, `moves`, `cut_length`, `cut_minutes`,
`max_tilt_deg`, `a_deg` when indexed, `gouge`, `holder_collisions`, `warnings`), `program_lines`,
`program_head` (the first lines), `readback`, `gouge`, `holder_collisions` (`null` when no tool
has holder data), `rapids` (`checked`, `hits`, `worst`, `pass`, `from_unknown`, `first`),
`travel` (`checked`, `violation`, `pass`; `null` without `linear_limits`), `stock` (as checked
against; `null` without one), `passed`, `failures`, `dry_run`, `outputs` (`gcode`, `preview`) and `notes`
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
| `stock` | The raw material in work coordinates: `{"box": {"min": [x, y, z], "max": [x, y, z]}}`, `{"cylinder": {"radius", "x_min", "x_max"}}` (a bar about work X, the A axis) or `{"around_part": {"margin": [x, y, z]}}` (the part's box grown by the margin each side). No rapid may pass through it; `face` faces it. Default: none. |
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

`number`, `kind` (`flat_endmill`, `ball_endmill`, `bull_nose`, `drill`, `thread_mill`),
`diameter`, `flute_length`, and optional `description`, `flutes`, `spindle_rpm`, `feed`,
`plunge_feed`, `stepdown`, `stepover`, `corner_radius` (bull nose), `shank_diameter`,
`stickout`, `holder: [{diameter, length}]`, `point_angle_deg` (a drill's point, default 118),
and for a thread mill `thread_pitch` and `tooth_height` (see
[Helical bores and thread milling](#helical-bores-and-thread-milling)). An operation without
`spindle_rpm` or `feeds` takes the tool's; its
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
| `drill` | `sketch` (circle centres, `circles` (true), filtered by `diameter_range: [min, max]`, and lone points, `sketch_points` (true)) or `points: [[x, y], …]`; `heights` (`retract` is the R height, `top` the hole's top, `bottom` its depth); `cycle`: `simple` \| `dwell` \| `peck` \| `chip_break` (simple); `peck` (tool's stepdown, else its diameter); `peck_clearance` (0.5); `back_off` (0.5); `dwell_seconds` (0.5); `to_full_diameter` (false); `stay_at_retract` (false). See [Drilling](#drilling). |
| `indexed` | `a_deg` and `operation`: any of the six above, run with A held at `a_deg` (3+1). Only these two keys; the inner operation carries the name, tool and feeds. |
| `rotary_finish` | `pattern`: `{type: "lines", angle_step_deg, zigzag?}` or `{type: "helix", pitch}`; `clearance` and `retract` (radii from the A axis); `floor` (0); `range: [min x, max x]` (the part's extent); `step`; `tolerance`; `stock_to_leave` (0); `max_angle_step_deg` (2) |
| `rotary_roughing` | As `rotary_finish`, with `stock_radius`, `stepdown` (in radius) and `stock_to_leave` (0.25) |
| `bore` | `sketch` (its circles) + `diameter_range?: [min, max]` or `holes: [{center: [x, y], diameter}]`; `heights` (`top` the hole's top, `bottom` its floor); `pitch` or `ramp_angle_deg` (2, the pitch at most `stepdown`: the tool's, else 10% of its diameter); `stepover` (between rings; the tool's, else 40% of its diameter); `climb` (true); `finish_passes` (0); `stock_to_leave` (0.1 with finishing passes, else 0) |
| `thread_mill` | `sketch` (its circles' centres) + `diameter_range?` or `centers: [[x, y], …]`; `heights` (the thread from `top` down to `bottom`); `pitch`; `thread_diameter` (the major, nominal diameter); `internal` (true); `right_hand` (true); `climb` (true); `thread_depth` (ISO metric: 0.5413 × pitch inside, 0.6134 × pitch outside); `radial_passes` (1); `spring_pass` (false) |
| `face` | `bottom`; `top` (the stock's top; required without a stock); `retract` (2 above the top, or the stock's top if higher); `clearance` (10 above the retract); `stepdown` (tool's, else 1); `stepover` (tool's, else 70% of the flat of its end); `angle_deg` (0); `zigzag` (true); `overhang` (half the tool's diameter and 0.5); `rect: [[x, y], [x, y]]` (the stock seen from above, else the part's extent). A flat end mill or a bull nose. |
| `wrap` | `radius` (of the cylinder the path is drawn on); `points: [[x, y, z], …]` (x along the axis, y the arc length round it, z the depth: 0 on the cylinder, negative into it) or `sketch` + `curve` (a curve id; its sketch x and y are taken as x and arc length) with `depth` (0); `clearance` (5) and `retract` (1) above the cylinder; `max_angle_step_deg` (2) |

The 3-axis operations (`contour` to `drill`) run with every rotary at zero; on `3axis` and
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
  tool's shank and holder (with holder data) must not touch them;
- with a stock, the cutter must not be in the stock. Nothing tracks what earlier operations
  removed, so the stock as given is the bound: a rapid below the stock's top over the stock
  fails, even where an earlier operation has cut it away, except after a `face` (with its
  default `overhang`) over the whole top of a box stock: the operations after it are held
  against the stock with its top at the face's `bottom`. A retract straight up the tool's
  axis out of the cut is exempt too: it only passes through room the tool already fills. Keep
  every retract and clearance height above the stock's top (on a 4th axis, outside its radius);
- a block that turns the A axis (a table about X) must keep the cutter outside the radius the
  stock sweeps (a box's farthest corner; without a stock, the part's largest radius and the
  tolerance) all through the turn, wherever the stock or part is level with the tool along X.
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
