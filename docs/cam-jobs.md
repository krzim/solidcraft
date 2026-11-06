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
| `rotary_finish` | `pattern`: `{type: "lines", angle_step_deg, zigzag?}`, `{type: "helix", pitch}` or `{type: "rings", stepover, zigzag?}` (below); `clearance` and `retract` (radii from the A axis); `floor` (0); `range: [min x, max x]` (the part's extent); `step`; `tolerance`; `stock_to_leave` (0); `max_angle_step_deg` (2) |
| `rotary_roughing` | As `rotary_finish`, with `stock_radius`, `stepdown` (in radius) and `stock_to_leave` (0.25) |
| `wrap` | `radius` (of the cylinder the path is drawn on); `points: [[x, y, z], …]` (x along the axis, y the arc length round it, z the depth: 0 on the cylinder, negative into it) or `sketch` + `curve` (a curve id; its sketch x and y are taken as x and arc length) with `depth` (0); `clearance` (5) and `retract` (1) above the cylinder; `max_angle_step_deg` (2) |
| `wrap_pocket` | `radius` (of the cylinder the profile is drawn on); `sketch` + `profiles?` or `polygon: [[x, arc length], …]` (unrolled: x along the axis, y the arc length round the cylinder); `bottom` (the floor's depth from the cylinder, negative into it); `top` (0); `stepdown`; `stepover`; `clearance` (5) and `retract` (1) above the cylinder; `climb`; `stock_to_leave` (walls); `ramp_angle_deg`; `tolerance`; `max_angle_step_deg` (2). See "Wrapped pockets and contours" below. |
| `wrap_contour` | As `wrap_pocket`, with `side` (outside) instead of `stepover`: the profile's boundary followed in depth levels |
| `indexed_pattern` | `a_deg: [angles]` or `count` (evenly over a turn, from `start_deg`, 0), and `operation`: one of the five 3-axis operations, run as an `indexed` operation at each angle, named `<name> A<angle>` |

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
