# NotoCAD Laser Backend — Specification

Nov 15, 2026 · @Sadie

## Purpose and scope

NotoCAD gains laser output as a second PLOT device: the same `draw_database` walk that feeds the PDF plotter feeds a `LaserRenderer`, and R12's colour-to-pen table becomes the laser layer table. Everything is C++ in the BSD-3 tree, driven from the command line and from files first; the Qt shell adds a laser viewport on top of the same calls.

**Version 1 delivers:**

- Vector cut and score jobs from the drawing (Extents, Window, Selection) to a Ruida 644x-class controller over UDP.
- A text job file (`.ljob`) and a binary Ruida file (`.rd`) for USB-stick and offline use.
- Frame, stop, status and lifetime counters, from inside NotoCAD and from a standalone `ncad-laser` tool.
- A driver interface that a second controller can implement without touching the front end.

**Later versions add:** raster engraving, SVG and AI/PDF import, and the GUI laser viewport.

**Out of scope:** galvo lasers, rotary axes, camera alignment, nesting, and any Python at runtime. MeerK40t is a source to port from and a test oracle, never a dependency of the shipped binary.

## Fit with NotoCAD

The backend reuses three things NotoCAD already has: the `Renderer` sink, PLOT's view and area logic, and the command engine. It follows the rules in `CLAUDE.md` rather than adding new ones. This section is based on `main` at 0.2.x; the `-solids` repository and Nsolid were not read.

&#91;embedded content: laser backend data flow · drawing to controller\]

The `LASER` command, `ncad-laser` and the GUI viewport all drive these same stages. Bitmaps skip the drawing and enter at the job as raster records.

**Where it hooks in**

- **`Renderer` (`render.hpp`).** Entities already emit world-space polylines, with curves flattened to `DrawContext::chord_tolerance`. A `LaserRenderer` collects those polylines per entity, using `begin_entity()` to resolve colour and layer into a pen. It needs no switch on entity type, so TEXT (Hershey strokes), SPLINE, ELLIPSE and block INSERTs all cut as drawn.
- **PLOT (`plot.hpp`).** `PlotArea` (Display, Extents, Limits, Window) and `PlotView` pick what goes to the laser exactly as they pick what goes on paper. The "plotted units = drawing units" question from PLOT's second phase is answered by default: a laser plots at 1:1 in mm.
- **The pen table.** R12's plot configuration assigned each ACI colour a pen number, pen speed and pen width. The laser pen table is the same table, with speed, power, passes and mode in place of pen fields. Building it here also delivers PLOT's planned second phase.
- **Commands.** `LASER` registers in `commands.cpp` like `PLOT`, prompts with R12 keyword syntax, and is drivable from `(command "LASER" ...)`. No laser function exists only in the window.

**Rules carried over**

- C++20 in the restrained dialect: single-level inheritance, no RTTI, status codes rather than exceptions in the encoder and planner.
- Headless first. Encoding, planning and file output build and test with no display and no network.
- Boundaries are visible in the build graph. Socket I/O lives in its own target, `ncad_laser`, behind `-DNCAD_WITH_LASER=ON` (default ON). The core never includes a socket header.
- BSD-3 default binary. Every third-party piece is permissive (MIT, zlib, Apache-2.0, public domain). Nothing GPL or AGPL is linked; see the import section for what that rules out.
- The `.ljob` text file is the inspectable interchange, the same way DXF text is for drawings. Tests compare bytes.

## Job format (.ljob)

A laser job is a controller-independent `LaserJob` struct with a line-oriented text form, `.ljob`. Every driver consumes it, and the planner reads and writes it, so a job can be built in NotoCAD, inspected or edited by hand, and sent later.

**Conventions**

- Millimetres, doubles. Job coordinates have the origin at the bed's lower-left corner, X right and Y up, the same as the drawing. The driver maps them to the machine's origin corner and axis flips, which come from the device config.
- Speeds in mm/s, power in percent with 0.1% resolution, frequency in Hz.
- A layer is identified by its pen number, which defaults to the ACI colour.

**Contents**

| Record | Fields |
| --- | --- |
| `ljob` | Format version, job name, source drawing, created time |
| `bed` | Width and height in mm, taken from the device |
| `layer` | Pen, ACI colour, mode (auto, vector, raster, skip), vector speed and power, raster speed, power, DPI and dither, passes, air assist, frequency, order |
| `path` | Layer, open or closed, then one `x y` pair per line, ended by `end` |
| `raster` | Layer, placement (x, y, width, height in mm), DPI, image file reference, dither method, bidirectional flag |

**Example**

```
ljob 1 name="tag-plate" source="tag.dxf"
bed 900 600
layer 5 color=5 mode=auto speed=150 power=18,18 passes=1 air=on order=1
layer 1 color=1 mode=auto speed=20 power=65,65 passes=1 air=on order=2
path 1 closed
  10.000 10.000
  90.000 10.000
  90.000 40.000
  10.000 40.000
end
raster 3 at=20,50 size=60,30 dpi=254 image="logo.png" dither=jarvis bidir=on
```

**Two stages of the same format.** The renderer writes a *raw* job: paths in drawing order, unjoined. The planner writes a *planned* job: paths joined and ordered. A `planned` flag in the header tells which, and drivers accept only planned jobs.

**Job origin is a fixed point, not a corner of the selection.** In LightBurn the user origin follows a corner, centre or edge-centre of whatever is selected, so cutting one layer or one selection at a time shifts the work unless a dummy mark below firing power pins the bounds. Here the origin is a point in the drawing, and every partial job is placed relative to it, so a layer cut now and another cut later line up exactly.

- **Where it lives.** A named UCS called `LASER`: its origin is the job origin and its X axis the machine X. It is saved in the DXF UCS table, picked with osnap like any UCS, and can be rotated to square the work to the sheet. With no `LASER` UCS, the origin is world 0,0.
- **Where it lands on the machine.** The `start=` setting chooses: `absolute` (origin at bed 0,0 after homing), `current` (origin at the head's position when Send runs), or a named position saved in `lasers.conf`.
- **Bounding-box anchoring** (LightBurn's behaviour) remains available as `start=bbox-<corner>` for one-off jobs, but is never the default.

## Commands and file workflow

Two front ends share one library. `LASER` works inside a drawing session. `ncad-laser` is a standalone tool for files, devices and cron jobs that never opens a drawing.

**`LASER` inside NotoCAD** (R12 keyword prompt)

```
Command: LASER
Device/Pens/Origin/Job/Order/Save/Send/Frame/Status/Counters/STop <Job>:
```

| Keyword | Does |
| --- | --- |
| Device | Select a configured device by name; sets `LASERDEV` |
| Pens | Edit or load the pen table; saves to a `.lpt` file |
| Job | Build a job from Display, Extents, Limits, Window or a selection set, then plan it; prints layers, path count, bounds and estimated time |
| Save | Write the current job as `.ljob` or `.rd` |
| Send | Bounds and power check, a confirmation prompt, then upload and run |
| Frame | Trace the job's bounding box with the laser off |
| Status | Controller state and position |
| Counters | Lifetime counters (power-on, job time, laser-on, travel) |
| STop | Abort the running job |

New system variables, all reachable through `getvar`/`setvar`: `LASERDEV` (device name), `LASERPENS` (pen table file), `LASERTOL` (chord tolerance, default 0.01 mm), `LASERCONFIRM` (1 = Send always asks).

**`ncad-laser`, the file-driven tool**

```
ncad-laser plan    in.ljob -o planned.ljob
ncad-laser encode  planned.ljob --device laser1 -o job.rd
ncad-laser send    planned.ljob --device laser1
ncad-laser frame   planned.ljob --device laser1
ncad-laser counters laser1 [--csv >> counters.csv]
ncad-laser rd2txt  job.rd            # decode any .rd, including LightBurn's
ncad-laser import  art.svg -o art.ljob --pens a2.lpt
```

`counters --csv` replaces the Python stats script with a cron-ready logger. `rd2txt` is the debugging tool every later stage depends on.

**Configuration files**

- `~/.config/ncad/lasers.conf`: one INI section per device with name, driver, host, model magic (or `auto`), bed size, origin corner, X/Y flip, maximum power, and acceleration for time estimates.
- `*.lpt` pen tables: the R12 `.pcp` idea, a plain text file per material and machine, so a pen table can be shared and kept in version control next to the drawings.

## Import pipeline

Vector imports become ordinary drawing entities, so the drawing stays the single source of truth and every path goes through the same pen table. Bitmaps are the exception: they live in the job, not the drawing, until NotoCAD has an IMAGE entity.

| Format | Library | License | Result |
| --- | --- | --- | --- |
| DXF | In-tree reader | BSD-3 | Already works; the usual LightBurn interchange |
| SVG | nanosvg (single header) | zlib | Cubic Béziers become SPLINE entities, exact (degree 3, clamped knots); lines become POLYLINE |
| AI (CS and later), PDF | qpdf for parsing and decompression; an in-tree interpreter for the path operators `m l c v y h re` | Apache-2.0 | Same Bézier path as SVG |
| AI before CS | None | — | Refused with a message; these files are PostScript |
| PNG, JPEG, BMP, GIF | stb\_image | Public domain or MIT | A raster record in the job |
| TIFF | libtiff, optional | libtiff (BSD-like) | A raster record in the job |

**Why not MuPDF or Poppler.** MuPDF is AGPL and Poppler is GPL. Either would make the binary GPL-family. The DWG precedent would allow one only as an opt-in compile-time module, and PDF import is too central to sit behind a flag. That conflicts with the default-build rule in `CLAUDE.md`. qpdf does the hard part, object parsing and FlateDecode, and leaves path interpretation, a few hundred lines, in-tree.

**Rules every importer follows**

- **Units.** SVG honours `viewBox` with width and height in mm or in; bare px is 96 per inch. Illustrator's px is 72 per inch, so an `--px-per-in` override exists. PDF user space is 1/72 in.
- **Colour.** Stroke colour maps to ACI by exact match first, then to the nearest palette entry. An optional map file can assign LightBurn's layer palette to pens.
- **Layers.** One NotoCAD layer per source layer or group where the format names them, so a SELECT by layer works after import.
- **Fills.** A stroke 0.5 mm or wider is kept as polyline width, and a fill becomes a filled region, so both engrave as raster under the geometry rule. Thinner strokes import as pure vectors.
- **Bitmaps.** Placement (x, y, width, height in mm) and DPI are stored in the `raster` record. The decoded pixels are never copied into the drawing.

## Job planning

The planner turns a raw job into a planned one in a fixed sequence. Each step is a pure function from `LaserJob` to `LaserJob`, tested on its own and exposed as `ncad-laser plan`.

**Vector or raster is decided by geometry.** Following LightBurn and the ULS print driver, thin lines cut or score and wide ones engrave. The threshold is 0.5 mm (about 0.020 in), which matches the physics: cutting a line about twice the kerf as two passes works but is messy.

- **Vector:** entities with width under 0.5 mm. That covers LINE, ARC, CIRCLE, ELLIPSE, SPLINE, Hershey TEXT strokes, and POLYLINE narrower than 0.5 mm.
- **Raster:** POLYLINE 0.5 mm or wider, TRACE, SOLID, filled regions from imports, and placed bitmaps. The renderer turns their area into a bitmap at the pen's raster DPI.

The pen table then supplies separate vector settings (speed, power, passes) and raster settings (speed, power, DPI, dither) for each colour. A pen's mode can be set to vector-only, raster-only or skip, which overrides the geometry rule for that colour.

1. **Flatten.** Already done by `draw()` at `LASERTOL`. The default is 0.01 mm, ten times finer than a typical kerf; Ruida coordinates are integers in µm.
2. **Join.** Chain open paths whose endpoints meet within 0.005 mm, and close a chain whose ends meet. DXF from other programs often arrives as separate LINE pieces, and an unjoined rectangle cuts as four starts and stops.
3. **Nest.** Build a containment tree of closed paths by point-in-polygon. Depth parity says hole or outer contour.
4. **No kerf compensation.** The beam cuts on the drawn line, as in every laser tool in use; allowing for kerf is the designer's job. Inside/outside compensation of the kind OMAX's waterjet software offers is deferred.
5. **Order layers.** By the pen table's `order`, with engrave and score before cut by default, so a part never drops out before its marking is done.
6. **Order paths within a layer.** Inner before outer, using the containment tree. Within one nesting level, nearest-neighbour from the current head position, then 2-opt on travel distance.
7. **Choose start points.** On a closed path, start at the vertex nearest the previous end. Prefer a corner over the middle of a straight run, where the start mark would show.
8. **Estimate time.** Trapezoidal motion per segment, using speed from the layer and acceleration from the device config, plus travel at the device's travel speed. The figure is reported, not guaranteed.

**Designer control of cut order**

The optimizer proposes an order, and the designer can see it, override any part of it, and have the override saved with the drawing. The optimizer then works around what the designer fixed. All of it is the `Order` keyword of `LASER`.

- **See it.** `LASER Order Show` numbers every path at its start point in the main viewport and draws travel moves dashed. `LASER Order List` prints the sequence with layer, length and travel, for terminal and script use.
- **Fix it.** `LASER Order Pick` takes entities in the order picked, with osnap and selection sets as usual. Picked entities keep that order; everything else is optimized around them.
- **Start point and direction.** `LASER Order Start` picks a point on a closed path, with osnap, as its start, and can reverse its direction. This places the start mark at a corner or where a tab will be.
- **Saved with the drawing.** Sequence number, start point and direction are stored as R12 XDATA under the application name `NCAD_LASER`, so they survive DXF save and load, and other programs pass them through.
- **Scriptable.** AutoLISP reads and sets the same XDATA with `entget` and `entmod`, so a script that generates geometry can generate its cut order too.
- **Clear it.** `LASER Order Clear` removes overrides from a selection, or from everything.

**What the optimizer weighs, beyond travel distance**

- Inner before outer, always. Overriding that warns, since a dropped part shifts everything cut after it.
- Finish one region before moving on, so parts are not left half-cut while the head works elsewhere.
- Avoid travel moves across parts already cut free, where a tipped-up part can catch the head.

ULS's optimizer is the benchmark. Which of its behaviours to match is an open question for the designer who has used it.

**Raster conversion (version 2)**

- Resample the image to the layer's line interval (25.4/DPI mm) at its placed size, convert to greyscale, then apply levels and gamma.
- Either dither (threshold, Floyd–Steinberg, Jarvis, Stucki, ordered 4×4) for on/off engraving, or map grey to power between the layer's min and max.
- Scan bidirectionally and skip blank lines and blank margins on each line.
- Add X overrun past both edges of the image so the head is at full scan speed before the first fired pixel and stays there past the last. Power is not modulated against acceleration. Overrun = v²/2a plus a margin, where a is the controller's own engraving acceleration read from its vendor settings (MeerK40t names `X Eng Acc` at 0x0125 and `Eng Acc Ratio` at 0x0137; units to confirm against the panel), falling back to the device config. MeerK40t's overscan of 0 is checked at M6, not assumed.

## Driver interface

A driver is split in two: an encoder that turns a planned job into bytes with no I/O, and a transport that moves bytes and answers queries. The encoder alone produces `.rd` files and runs in every test; the transport is the only code that touches a socket.

```cpp
// include/ncad/laser/driver.hpp  (core; no socket headers)
struct LaserCaps {
    double bed_w_mm, bed_h_mm;
    bool   curves;          // accepts arcs or Béziers; false = segments only
    bool   raster;          // has a raster encoder
    bool   needs_overscan;  // planner must add acceleration space
    double max_power_pct;
    double coord_step_mm;   // 0.001 for Ruida
};

class LaserEncoder {        // pure: job in, bytes out
public:
    virtual ~LaserEncoder() = default;
    virtual LaserCaps caps() const = 0;
    virtual Status encode(const LaserJob& planned, std::string& out,
                          std::string& error) = 0;
};

class LaserTransport {      // ncad_laser target only
public:
    virtual ~LaserTransport() = default;
    virtual Status open(const DeviceConfig&, std::string& error) = 0;
    virtual Status send(std::string_view job, Progress&, std::string& error) = 0;
    virtual Status frame(const BBox& job_box, std::string& error) = 0;
    virtual Status stop(std::string& error) = 0;
    virtual Status status(MachineStatus&, std::string& error) = 0;
    virtual Status counters(Counters&, std::string& error) = 0;
};
```

**Rules**

- Drivers register by name in a table like the command registry. `lasers.conf` names the driver; no front-end code names a controller.
- `Status` codes, no exceptions, no RTTI. `Progress` is a callback struct so the terminal prints a percentage and the GUI updates a bar from a worker thread.
- The planner reads `LaserCaps`, never a driver type. A segments-only driver gets flattened paths; a curve-capable one could get Béziers in a later version.
- `Counters` is a fixed struct: power-on time, job time, laser-on time, job count, travel per axis, card ID, firmware version. A driver leaves a field unset if its controller lacks it.

**Drivers planned**

| Driver | Purpose | When |
| --- | --- | --- |
| `ruida` | A2's machines over UDP, and `.rd` files | Version 1 |
| `file` | Encoder output only, no transport | Version 1 |
| `grbl` | G-code text; proves the interface isn't Ruida-shaped | Version 1.5 |
| MeerK40t bridge | Hand a planned job to MeerK40t for its other controllers | Deferred |

## Ruida driver

The Ruida driver is a C++ port of MeerK40t's `meerk40t/ruida` encoder and UDP framing, checked against `.rd` files that LightBurn writes for A2's own machines. MeerK40t's README says its Ruida support was tested on one controller (RDC6442S) and needs Flip X enabled, so it is a strong starting point but not the final authority.

**Transport (UDP)**

- Send to controller port 50200 from local port 40200. Each packet is a big-endian 16-bit checksum followed by the swizzled payload. The checksum is the sum of the *swizzled* bytes, as in MeerK40t's `udp_connection.py` (`_package`).
- Swizzle and unswizzle with the model's magic byte: 0x88 for most 644x boards. `magic=auto` reads the card ID at memory address `05 7E` and looks the magic up.
- Every packet waits for an ACK (`0xCC`); a NAK (`0xCF`) resends that packet. The timeout is about 4 s. Chunks stay under about 1000 bytes and split at command boundaries, never inside one.
- Jog and frame can use the unswizzled keypad protocol on port 50207 (`A5 50 xx` key down, `A5 51 xx` key up) where a command form is not available.

**Encoder**

- Numbers are 7 bits per byte. Coordinates are signed 35-bit integers in µm, power is 14-bit, and speed, frequency and time are 35-bit.
- The job header follows `RDJob.write_header`: process bounds, then per part (layer) the speed, frequency, min and max power for both tubes, colour, and part bounds, then array info. Layer settings follow `write_settings`; the tail follows `write_tail`, which sets the file sum (plus 0xD7 for the end-of-file byte) and then writes the EOF byte.
- Moves: use a relative move when both deltas are within ±8.192 mm, otherwise an absolute move. The first move of a file is always absolute; MeerK40t records a fault otherwise.
- The machine transform (origin corner, X/Y flip, bed size) is applied here, after planning, so `.ljob` files never depend on the machine. The start mode (absolute, current position, saved position) selects the RD reference-point commands. MeerK40t's ref\_point calls are the starting point, checked against LightBurn .rd files saved in each Start From mode.

**Counters**

Read with `DA 00 hi lo`; the reply is `DA 01 hi lo` followed by a 35-bit value. The addresses are those used by the stats script: `04 01` power-on time, `04 02` job time, `04 11` laser-on time, `04 03` job count, `04 23` and `04 33` travel on axes 1 and 2, `05 7E` card ID, `05 7F` firmware. MeerK40t's `RuidaEmulator.mem_lookup` has the fuller table and is the reference when adding fields.

**What is taken from MeerK40t**

| MeerK40t file | Taken | Use |
| --- | --- | --- |
| `rdjob.py` | Value encoders, swizzle tables, header, layer, tail, jump and mark logic | Ported to `src/laser/ruida_encoder.cpp` |
| `udp_connection.py` | Packet framing, checksum, ACK and resend loop | Ported to `src/laser/ruida_udp.cpp` |
| `emulator.py` | `mem_lookup` names; the command parser | Memory table; test oracle only |
| `rdjob.py` parser | Command decoding | Basis for `ncad-laser rd2txt` |

**Licensing.** MeerK40t is MIT (Copyright (c) 2021 meerk40t), which may be combined into a BSD-3 project. Each ported file carries `SPDX-License-Identifier: BSD-3-Clause AND MIT` and both copyright lines, and the MIT text goes in `third_party/meerk40t/NOTICE`, following the Hershey font's pattern.

## GUI laser viewport

The laser viewport is a separate view in the Qt shell, showing the machine bed rather than model space. It adds no laser behaviour of its own: every button issues a `LASER` command string through `CommandEngine`, so the command line shows what happened and `(command "LASER" ...)` can do the same.

**What it shows**

- The bed as a fixed rectangle in mm, with the origin and home corner marked. It opens fitted to the bed and pans and zooms like the main viewport.
- The planned job, drawn from the `.ljob`: cut paths coloured by pen, travel moves dashed, and an optional order number at each path start.
- Raster records as placed images at their real size, which is where bitmaps finally have a home.
- Job bounds, estimated time, and a red outline if any part falls off the bed.

**What it edits**

- The pen table, as a grid with one row per pen: speed, power, passes, mode, order, air. It writes the same `.lpt` file that `LASER Pens` reads.
- Job placement on the bed (move, and later rotate). Placement is stored in the job, not the drawing.

**Controls**

Frame, Send, Stop and Status, plus a preview slider that replays the planned order. Send and Frame run on a worker thread through the `Progress` callback; the UI never blocks on a UDP wait. Stop is always enabled while a job runs.

**Implementation notes**

- `QPainter`, like the main viewport: the content is polylines and images, so no GL is needed.
- The viewport draws from a `LaserJob`, not from the entity database, which keeps it independent of the drawing's model space and UCS.
- It lives under `src/gui/` and links only `ncad_laser` plus Qt. The dynamic-Qt rule in that CMakeLists applies unchanged.

## Safety, testing and milestones

Nothing reaches a tube until the encoder matches LightBurn's output for the same design, and the first powered run is a low-power score on card.

**Safety rules in the code**

- Only an explicit Send moves the head under power. Job, Save, Frame and Counters never fire the tube.
- Send refuses a job that leaves the bed or exceeds the device's `max_power`, then shows layers, bounds and time and asks for confirmation (`LASERCONFIRM`).
- Stop is reachable from both front ends at all times, and from `ncad-laser stop` in a second terminal.
- The controller accepts unauthenticated UDP, so `lasers.conf` documents the expected VLAN and firewall rule rather than pretending the protocol is secure.

**Testing**

| Layer | Test | Oracle |
| --- | --- | --- |
| Encoder | Golden-byte tests per command and per small job | Hand-checked bytes, then MeerK40t's encoder output |
| Encoder | Decode our `.rd` with MeerK40t's parser (an optional CI job, Python only in CI) | MeerK40t |
| Encoder | Same design saved as `.rd` from LightBurn, both decoded by `rd2txt`, structure compared | LightBurn on A2's machines |
| Planner | Join, nest and order on fixture drawings | Expected `.ljob` text |
| Transport | Against MeerK40t's emulator over loopback UDP | Emulator log |
| Hardware | Counters, then frame, then score on card at about 10% power, then a cut | A2 laser |

**Milestones**

&#91;embedded content: milestones M0–M6 · each with its gate\]

M0 and M1 need no powered laser and can be done entirely against files and LightBurn's output; M2 is the first run under power.

**Decisions (15 Nov 2026)**

- **Run from the code first.** Send uploads and runs immediately. Storing a job on the controller for a later run from the panel comes after M2.
- **Firmware.** All three A2 lasers run 8.01.50, so one encoder profile covers the fleet. Magic 0x88 is confirmed by the counters read.
- **Raster overrun.** Raster runs at high traverse speed, so the planner adds X overrun past both image edges and fires only at full speed; no power compensation during acceleration. The overrun comes from the controller's own acceleration limits.
- **Imports are native.** SVG and AI/PDF are read into native entities, whatever the source, and show in the viewport like anything drawn.
- **Rasters.** At first, a raster is placed only in the laser viewport and stored in the job. An IMAGE entity in R2000 DXF is the later home.
- **MeerK40t bridge.** Deferred; not a priority.

* **Vector threshold.** Lines under 0.5 mm (about 0.020 in) cut as vectors; wider ones engrave.
* **Kerf.** Not compensated. It is the designer's job, as in every laser tool in use; OMAX-style inside/outside choice is deferred.

- **Job origin.** A fixed point, the `LASER` named UCS, not a corner of the selection, so partial jobs register with each other without a sub-threshold dummy mark.

**Raster encoding: learn it from LightBurn**

- Save raster test images (a greyscale wedge, a dithered photo, a 1-pixel line pattern) as `.rd` from LightBurn on an A2 machine, and decode them with `rd2txt`.
- Diff those against MeerK40t's raster encoding of the same images, then confirm the differences on the machine at low power.
- LightBurn writes `.rd`, not G-code, for a Ruida, so the `.rd` files are the reference.
