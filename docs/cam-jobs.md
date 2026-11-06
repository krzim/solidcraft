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

## Profiles, defaults and refusals

These are what let an agent (or a person) go from a modelled part to checked G-code in one or
two calls: the machine's setup lives in a profile, the numbers that follow from the part are
worked out from it, and every refusal says what to change.

```json
{"job": {"profile_path": "C:/cam/profiles/mill-3axis.profile.json",
         "operations": [{"name": "pocket", "type": "pocket", "tool": {"kind": "flat_endmill", "max_diameter": 6},
                         "polygon": [[-10, -6], [10, -6], [10, 6], [-10, 6]]}]},
 "output": "C:/cam/part.tap", "compact": true}
```

### Profiles

`profile_path` names a JSON file with what every job on one machine shares. Its keys are a
job's (`machine` or `machine_path`, `a_reversed`, `a_limits_deg`, `post` or `post_path`,
`work_offset`, `tolerance`, `origin`, `fixtures`, `tools`, `tool_library_path`,
`tool_library_paths`, `tool_library`, `default_heights`, `default_feeds`) plus `name` and
`description`, which are not written anywhere; it refuses keys it does not know. Relative paths
in it are read from its own folder; a relative `profile_path` in a job file from the job's.

The job's own keys win: a job's `machine` or `machine_path` replaces the profile's machine and
its A-axis options, a job's `post` or `post_path` the profile's post, and a job's `tools`,
`tool_library_path` or `tool_library_paths` its tools. `default_heights` and `default_feeds` are
merged key by key.

| Key | Meaning |
| --- | --- |
| `machine_path` | The machine as a `Machine` JSON file (a job may give it too). |
| `tool_library_paths` | Several tool library files. A tool number found in two of them with different tools is refused with the clashing numbers listed, unless the job picks one library with `tool_library` (its path, file name, or file name without `.json`). The same tool in two libraries is no clash. A library tool this build cannot read (a kind or key it does not know) is left out with a note, and an operation that asks for its number is told why. |
| `default_heights` | `{clearance_above, retract_above}` (10 and 2 mm): how far above the part the heights an operation leaves out go. |
| `default_feeds` | `{cut_scale?, plunge_scale?, max_cut?, max_plunge?, spindle_rpm_range?: [min, max], fallback?: {cut, plunge, spindle_rpm}}`. A tool's feeds are scaled (0.05 to 2) and then held to the limits, and the reply's notes say so; an operation's own `feeds` or `spindle_rpm` beyond a limit is refused. `fallback` is for tools without cutting data. |

### Heights worked out from the part

A `contour`, `pocket`, `parallel`, `roughing` or `waterline` (also inside `indexed`) may leave
`heights` out, or any of its four keys. In the operation's frame:

- `top`: the part's top, from the bodies' exact boxes. In an indexed frame whose axes are not
  along the design's, the box's corners are turned into the frame, which puts `top` at or
  above the part.
- `bottom`: for a `pocket` and an `inside` or `on` contour, the lowest floor of the part inside
  the profiles: the exact B-rep is classified down columns spread over the region (flat floors
  found exactly at their face's height, curved ones within a quarter of the tolerance, from
  above), and the part's bottom where the region goes through the part. For an `outside`
  contour and the 3D operations, the part's bottom. A region where the part is solid up to
  the top is refused: there is nothing to cut.
- `retract`: `top` + `default_heights.retract_above` (2).
- `clearance`: the higher of `top`, the part's top and the fixtures' top, plus
  `default_heights.clearance_above` (10), and at least `retract`.

The reply's `derived_heights` lists each such operation's heights, which were worked out and
from what. Given or worked out, a clearance at or below the part's or fixtures' top is refused
when that top is exact.

### Tools by what they are

An operation's `tool` may be `{kind?, diameter?, max_diameter?, min_diameter?,
min_flute_length?, min_stickout?}` instead of a number. The tools that meet every key match (with
no `kind`, any tool but a drill); a `pocket` or an `inside` contour also needs the tool to fit its
profiles. Of the matches, one of exactly `diameter` if given, else the largest diameter, then the
shortest flutes, then the lowest number is picked. The reply's `tool_choices` gives every
operation's tool and why (`given by number`, or the rule and the other matches).

### Refusals

Every refusal names the operation (its index in `operations`, from 0, and its name), the check
(`heights`, `clearance`, `tool`, `feeds`, `fit`, `region`, `plan`, `gouge`, `holder`, `readback`,
`tools`), the place in work coordinates and the program line when there is one, and a fix: the
widest tool that fits a pocket, the clearance that clears the part, the stickout a holder
collision needs, the library to pick on a tool-number clash. All the operations are planned
before a refusal, so one call reports every operation that cannot be planned. The message has a
line per problem and ends with `refusals: ` and the same as JSON,
`[{error, op: {index, name} | null, check, at: [x, y, z] | null, line?, fix}]`; the MCP server
returns it as the error's `structuredContent.cam_refusals`. A dry run's reply has them as
`failure_details`.

### Compact replies

`compact: true` replies `{compact: true, passed, dry_run, failures, operations, tools,
total_minutes, program_lines, outputs}`: the failures with their fixes, one line per operation
(index, name, kind, tool, moves, minutes, heights worked out, gouges, warnings), every
operation's tool and why, and the program's time at feed as the controller times it.
