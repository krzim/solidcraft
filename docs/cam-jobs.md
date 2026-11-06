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

## Machinable features and suggested jobs

Two commands let an agent go from a modelled part to a checked program without drawing a sketch
for every cut or guessing a single height:

| Command | What it does |
| --- | --- |
| `cam.features` | Lists the part's machinable features with stable ids and their exact geometry |
| `cam.suggest` | Proposes a whole job from those features and a tool library, for `cam.job` to check |

A short session: `cam.features {}` to see what there is, `cam.suggest {"profile_path": "my-mill.json"}`
for a job, `cam.job {"job": …, "dry_run": true}` to check it, then `cam.job {"job": …, "output":
"part.tap", "preview": "part.html"}`. Every refusal names the key or value to change.

### `cam.features`

```json
{"bodies": ["Plate"], "origin": [0, 0, 0], "axis": "z", "tolerance": 0.01}
```

Every key is optional: the bodies default to every visible body, `origin` to the design's origin
(the job's work zero), `axis` to `"z"` and `tolerance` to 0.01 mm (how close two sizes or
positions must be to count as equal). With `axis: "a"` (a 4th axis about work X) every side whose
normal is square to X is a setup of its own, at the A angle the machine turns to bring that side
up, so A = 0 is the 3-axis setup and the other sides follow it round.

Features are read from the exact B-rep, never from a mesh: the faces' analytic surfaces, their
loops of edges and the faces on the other side of each edge, lines and arcs read exactly in the
setup's plane. Every coordinate of a feature is in its setup's frame (work coordinates turned by
the setup's A angle: its Z is the tool axis), so its numbers go straight into an operation of
that setup.

| List | Each feature |
| --- | --- |
| `setups` | `a_deg`, `tool_axis` (work), `top_z`, `bottom_z`, `bounds`, `swing_radius` (axis `a`), `features` (the ids machined from that side) |
| `holes` | `hole:k`: `center`, `center_work`, `diameter`, `top_z`, `bottom_z`, `depth`, `through`, `floor` (`through`, `flat`, `cone` with `tip_z`, or `round`: the wall runs on into a curved floor within its radius, below `bottom_z`), `counterbore` / `counterbore_of` (the hole above or below it on the same axis), `countersink` (`diameter`, `angle_deg`), `faces` |
| `hole_groups` | holes of one diameter, depth and kind: `diameter`, `depth`, `through`, `count`, `ids` |
| `pockets` | `pocket:k`: a planar floor walled all round. `floor_z`, `top_z` (the top of its walls, followed up past a round or chamfer at their foot), `depth`, `outline` (counter-clockwise), `area` (less its islands), `bounds`, `min_corner_radius` (the smallest inside corner; 0 when one is sharp), `sharp_corners`, `walls_vertical` (false when the walls are rounded, chamfered or drafted at the floor: the outline is then the floor's, inside the walls), `islands` (`outline`, `top_z` the top of the island, past a round at its foot, `area`), `face_id` |
| `faces` | `face:k`: every planar face facing the tool, with `z`, `area`, `bounds`, `normal_work` and `role`: `top`, `plateau` (everything round it falls away), `step` (open on some sides), `island_top`, `pocket_floor` (with `pocket`), `hole_floor` (a blind hole's floor or a counterbore's step) |
| `profiles` | `profile:k`: the outline of a face whose walls fall straight away all round, and the outline of the bottom face (the silhouette) when no face above gives the same: `top_z`, `bottom_z`, `outline`, `min_concave_radius` (its notches, where a tool running outside must fit) |
| `fillets`, `chamfers` | `fillet:k`, `chamfer:k`: rounds and chamfers falling away from the edges of faces facing the tool, `radius` or `angle_deg` (from the tool axis), `top_z`, `bottom_z`, `bounds`, `faces` |
| `surfaces` | `surface:k`: other curved faces, joined where they touch: `kinds`, `min_radius`, `top_z`, `bottom_z`, `bounds` |
| `turned` | `turned:k` (axis `a`): a diameter turned about the A axis, `diameter`, `x_min`, `x_max` |

Ids are numbered per kind in a fixed order: the A = 0 setup first, then from the top down, then by
position (Y, then X). The same bodies, origin, axis and tolerance give the same ids, and the A = 0
features have the same ids with `axis: "z"` and `axis: "a"`. `notes` says what was left out: mesh
bodies, holes square to the tool axis on a 3-axis mill, sides a 4th axis cannot reach.

A hole is a whole round wall along the tool axis with material outside it, open at its top (the
part of a round wall that is a pocket's corner or a slot's end is a wall, not a hole). Whether a
round wall is a hole or a boss is read from the planar faces at its ends (the wall leaving an
inner loop of a face into the material, or rising from a floor's outer loop), else by
classifying a point just outside it on the exact solid.

### Operations on features

A `pocket` or `contour` operation takes `feature` in place of `sketch` or `polygon`:

```json
{"name": "pocket", "type": "pocket", "feature": "pocket:0", "tool": 2}
```

| Feature | Region | Heights (`top`, `bottom`) | `side` of a contour |
| --- | --- | --- | --- |
| `pocket:k` | its outline, with every island that rises above the operation's bottom | its highest wall, its floor | inside |
| `hole:k` | its circle | its top, its bottom (a through hole: the part's bottom face) | inside |
| `face:k` | its outline, with what stands on it (a pocket on a face at the setup's top covers the face's extent grown by the tool's radius and 1 mm, so it faces to the edges) | the walls round it (a top face: none, so give `heights.top`), the face | inside |
| `profile:k` | its outline | its face, the foot of its walls | outside |

`heights` may then be left out or given in part (`{"bottom": -3}`): what is left out comes from
the feature and from the heights given: `retract` 3 mm above the material (the part's top in the
feature's setup, or the `top` given when higher), `clearance` 10 mm above the material and at
least 5 mm above the `retract` given, 5 mm past the part's swing radius on a 4th axis and 5 mm
above the job's `fixtures` (the corners of their exact boxes in the setup's frame). Heights out of
order are refused with each value and whether it was given or a default. Without a feature all four
heights are still required. The job finds features as `cam.features` does with the job's
`bodies` (else every visible body not in `fixtures`), `origin` and `tolerance`, along `a` when the
machine has one rotary about X, else `z`.

A feature on another side of a 4th-axis part carries its A angle: named outside an `indexed`
operation, the operation is indexed there itself (the reply's `kind` is `indexed pocket` and its
`a_deg` the feature's); inside an `indexed` operation at another angle it is refused with the
angle to use. Fillets, chamfers, surfaces and turned diameters have no outline: a 2D operation on
one is refused with the `boundary` rectangle and floor to give a `parallel` or `waterline`
operation instead. Drilling and boring operations take `features: ["hole:0", "hole:1"]` through
the same lookup.

### `cam.suggest`

```json
{"profile_path": "C:/cam/my-mill.json", "axis": "a", "stock_top": 0.5, "check": true}
```

| Key | Meaning |
| --- | --- |
| `profile_path` | A machine profile: a JSON array of tools (a tool library), or an object with `tools` or `tool_library_path` and optionally `machine` or `machine_path` (a file holding the machine JSON), `post` or `post_path`, `a_reversed`, `a_limits_deg`, `work_offset` (as a job has them; relative paths are read from the file's folder). Give this, `tools` or `tool_library_path`. |
| `tools`, `tool_library_path` | The tools to choose from, as a job has them. |
| `bodies`, `origin`, `tolerance` | As `cam.features`; they are copied into the job. |
| `axis` | `z` or `a`. Default: `a` when the profile's machine has one rotary about X. |
| `post` | A built-in post. Default: the profile's, else `linuxcnc_4axis`. |
| `stock_top` | Material faced off the top, 0 to 5 mm (default 0.5; 0: no facing). |
| `check` | Also run the job through `cam.job` as a dry run and return its report as `dry_run`. |

The reply: `job` (a complete job: name, machine, post, origin, bodies, tolerance, only the tools it
uses, and its operations, which name features), `operations` (each with the features it machines
and why it was chosen), `tools` (what each is used for), `assumptions`, `check` (what to verify
before running the program), `skipped` (features left out, tools that could not be used, and why)
and, with `check`, `dry_run`. Nothing is written.

Only tools with `spindle_rpm` and `feed` are picked. Each pick is tried with the operation itself
on the feature's exact outline, so a tool is never proposed for a pocket it does not fit. The
operations, in order:

1. **Facing** (3-axis, when the top is flat and `stock_top` is above 0): a pocket over the part's
   extent and 1 mm round it, `stock_top` deep, with the largest flat end mill.
2. **3D roughing** (curved surfaces only): the whole part down to the lowest curved area, 0.25 mm
   left, with the largest flat end mill.
3. **Pockets**, roughed with the largest flat end mill the pocket takes (flutes as long as the
   depth preferred), 0.2 mm left on the walls; islands lower than the rim in bands of their own.
4. **Holes**, counterbores first, milled as round pockets with the largest flat end mill of at most
   80% of the hole's diameter. A hole no tool fits is skipped, with the largest diameter that would
   fit and the operation to add; a drill of its diameter in the library is named, but `cam.job` has
   no drilling operation to use it yet.
5. **Pocket walls** finished with a contour by the largest flat end mill whose radius is at most
   the pocket's `min_corner_radius` (when a corner is sharp: the smallest that fits, with a check
   saying what it leaves).
6. **3D finishing** of surfaces, fillets and chamfers: a parallel finish over their bounds with the
   largest ball end mill whose radius is at most their smallest radius (else the smallest ball,
   else a bull nose, else a flat end mill).
7. **Turned diameters** (axis `a`): a rotary finish along their X range.
8. **Outer profiles** (3-axis), last: a contour outside with the largest flat end mill that fits
   their notches, flutes as long as the depth preferred.

Pockets and holes on other sides of a 4th-axis part are `indexed` at their A angles; facing,
profiles and 3D finishing are 3-axis only. Steps (faces open on some sides) are listed under
`skipped` with the operation to cut them (a pocket on a step stays inside its outline: give a
`polygon` past its open sides to clear them). A pocket whose walls are not upright gets a `check`
line saying its walls are left, with the bounds for a 3D finish. Always run the job with `dry_run: true` first; the
suggestion is tested to pass on a plate with a pocket, an island and holes, and on a bar with a
pocket on each side.
