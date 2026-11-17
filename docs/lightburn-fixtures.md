# LightBurn fixtures: designs to round-trip

Each fixture is a small DXF, imported into LightBurn and saved as `.rd` (and
`.lbrn2`, for the later job-definition work). Each answers specific questions
about what LightBurn writes. Running the same DXF through `ncad-laser` and
diffing the two with `rd2txt --structure` turns every answer into a test.

## Common rules

- **DXF:** R12, millimetres (`$INSUNITS` 4), geometry near 0,0, every
  entity on a layer named for its pen and coloured BYLAYER.
- **LightBurn:** your normal device profile for Bengal (say if another is
  used). Record anything you change from your usual settings in a line of
  `notes.txt` next to the files.
- **Save:** `File > Save RD File` without running; also `File > Save` for
  `.lbrn2`. Name both after the fixture: `F03-start-absolute.rd`,
  `F03-start-absolute.lbrn2`.
- **Defaults unless a fixture says otherwise:** Start From = Current
  Position, Job Origin = bottom-left, one pass, air on, LightBurn's
  optimisation settings left as you normally have them.
- Put the files in `tests/fixtures/lightburn/`.

## The fixtures

| # | Design | LightBurn settings | Answers |
|---|---|---|---|
| F01 | 40 mm square, layer 1 | 100 mm/s, 10/15 % | The baseline everything else is diffed against |
| F02 | F01 | Passes = 2; then also save with 3 | How passes are encoded: repeated blocks, or a count? |
| F03 | F01 | Start From = Absolute Coords | The ref-point command and header for absolute mode |
| F04 | F01 | Start From = User Origin | The same for user origin |
| F05 | F01 | Job origin at **centre** (the middle dot of the 9) | Whether the header accepts an origin that is not a corner; answers the fixed-origin question without hardware |
| F06 | F01, saved 4 times | Job origin top-right, top-left, bottom-right, and middle-left | Confirms the origin rule across corners and an edge midpoint |
| F07 | Circles of 2, 10, 50 and 200 mm diameter, one layer | — | How finely LightBurn flattens arcs (Ruida has no arc command, as far as we know), so our chord tolerance can match |
| F08 | One SPLINE (S-curve, about 60 mm) and one 3-point ARC | — | Curve flattening for imported curves |
| F09 | A 100 mm line, a 0.5 mm line, a polyline of 200 steps of 0.05 mm, and a 9 mm jump between two short lines | — | Relative vs absolute choice at the 8.192 mm limit; very small segments |
| F10 | Square 60 mm with a 20 mm square hole and a 5 mm circle inside the hole | Cut inner first on (if not already) | LightBurn's inner-before-outer order and start points, to compare with our planner |
| F11 | Same square on 3 layers (ACI 1, 3, 5), side by side | Layer 1: 20 mm/s 60/65 %, air on; layer 3: 150 mm/s 18/18 %, air off; layer 5: skip (output off) | Per-layer settings, air off, a disabled layer, layer order |
| F12 | One 10 mm square for each ACI colour 1–9, 10, 30, 50, 130, 250 | Import only, no setting changes | Which LightBurn layer each ACI colour lands on, for the import colour map |
| F13 | F01 | Min power 10, max 40; speed 300 mm/s | That min and max power stay separate through the header and layer block |
| F14 | F01 | Second laser tube (if the machine has one): set different powers for tube 2 | What tube-2 settings look like; skip if single tube |

### Raster, for later (version 2), if convenient while LightBurn is open

| # | Design | Settings | Answers |
|---|---|---|---|
| R01 | A 256-step greyscale wedge, 50 × 10 mm (PNG) | Image mode Greyscale, 254 DPI | Raster encoding, power-per-pixel |
| R02 | Same wedge | Dither (Jarvis), 254 DPI | Dithered encoding |
| R03 | A 1-pixel line pattern | Threshold | Overscan, scan direction, blank-line skipping |

## Priority

If time is short: F01, F02, F03, F05 and F07. Between them they cover
passes, start modes, the non-corner origin and arc flattening, which are the
open questions blocking the encoder.

## Findings (2026-11-16)

Files: tests/fixtures/lightburn/ (notes.txt records the user's notes). Pinned by tests/test_lightburn_fixtures.cpp.

- Layout (RdLayout::Current; the older files are RdLayout::Legacy): a `Speed Axis 15mm/s` before every `Layer End`, and every pass of every path starts with a short block `Work Mode 6, Speed Axis, Layer End, Layer Number, En Laser 2 Offset, Laser Device 0, Tube Start 3` followed by the full settings block (`... Tube Start 1`) and the cuts. The layer's first Move follows its first settings block; a later path's Move comes AFTER its first pass's two blocks, just before the cuts (f07: `F Move SF cuts SF cuts SF Move cuts SF cuts ...`). The axis speed is `RdJob::axis_speed_mm_s` (15 in every file). f01, f02-p2/p3, f05, f06-BR and f07 are all reproduced byte for byte by `check_reproduces` (tests/test_ruida_encode.cpp).
- Passes (f01 vs f02-p2 vs f02-p3): each extra pass repeats the short block, the settings block and the cuts; the Move is written once per path (f07: per circle, so passes are per shape, not per layer). File size 625 / 764 / 903 bytes (+139 per pass). Whether the first cut after a block is absolute is decided by the 8.192 mm rule alone (f07 goes relative straight after its blocks); only the layer's first motion is forced absolute. No sample has an OPEN path with passes > 1 or passes with several layers: the encoder travels back to an open path's start before each further pass, and repeats the scheme per layer. Both are guesses, flagged in ruida_encode.hpp.
- The cut counter `06 20` (160 / 320 / 480 for the square) is the MANHATTAN length of the cuts (sum of |dx|+|dy|, floored to mm), not the Euclidean one: f07 reads 2096 = 2 passes x 8 r x (1+5+25+100), against a Euclidean 1646.3. The squares cannot tell the two apart.
- f05, f06-BR and f07 were saved with Passes still at 2 (8 cuts, 764 bytes; f07 traces each circle twice in one polyline).
- Centre origin (f05): process box is -20..20 both axes; `origin = Document Min Point - Process Top Left` still holds (5.2, 31.2 in every file, f03 included; see F03 for what that number probably is); summarize_rd reports "other".
- Corner names: the decoder names corners in file coordinates, 180 degrees from LightBurn's label. f01 (LightBurn top-left) is box x -W..0, y 0..H, reported "bottom-right"; f06-BR (LightBurn bottom-right) is box x 0..W, y -H..0, reported "top-left". Same rule, mirrored in both axes.
- Arcs (f07): per pass 32 / 64 / 128 / 256 segments for D2 / D10 / D50 / D200, i.e. the smallest power of two with sagitta R(1-cos(pi/n)) <= tolerance; measured max sagitta 0.0082 mm. Tolerance lies in [0.0075, 0.0192) mm, probably 0.01 mm. Match this in the planner / dxf2ljob default later (not changed now). Vertices are not evenly spaced (chord varies about 7%).
- How LightBurn flattens a circle (f07, reverse-engineered; `dxf2ljob --flatten lightburn`): four cubic Beziers, one per quadrant, k = 4/3 tan(pi/8) = 0.5522847498, each cut at uniform t into n/4 steps (n = smallest power of two, at least 8, with R(1-cos(pi/n)) <= 0.01). That explains the ~7% spacing. Direction: clockwise in the drawing, from the 180 degree point (vertices fall on 180/90/0/-90 and, at the mid-parameter, 45 degrees). The cut loop is then rotated to a start vertex. Reproduced to within 0.0006 mm of f07's vertices (its own 1 um rounding). ARC and ELLIPSE use the same construction (quarter-turn Beziers, steps scaled by sweep) but no fixture checks them.
- Order and head (f07): the four circles are cut D50, D10, D2, D200 = plain nearest-neighbour from the BEAM, which in design terms is the job origin shifted by the pointer offset ((0,200)+(5.2,31.2) = (5.2,231.2) here), each next one from where the last ended. `plan --head X,Y --no-2opt` does that; our default 2-opt plan differs.
- Start vertices (f07), NOT explained: LightBurn starts D50 at 106.9 deg, D10 at 56.0, D2 at 22.9 and D200 at 180 (design angles; 26/128, 22/64, 14/32 and 0/256 of the way round from 180 deg). Against the vertex nearest the previous end the start is 2 after, 4 before, 1 before and the same vertex; not nearest to either end, nor by in+out travel, nor in any of Manhattan, Chebyshev or axis metrics. So the planner keeps its own rule and the parity test takes the start from the file.
- Parity (tests/test_laser_renderer.cpp, "lightburn parity: F07"): with the file's start vertices, DXF -> `--flatten lightburn` -> plan -> encode reproduces f07.rd except that ~13% of the vertices are 1 um off (LightBurn's vertices are up to ~0.09 um from the exact Bezier points; not float32, not k). Same size, same commands, same header.

### F03: Start From = Absolute Coords

f03 (same 40 mm square, top-right machine, Passes 1; tests/fixtures/lightburn/f03.rd and .lbrn2) differs from f01 in the header opening and in coordinates only. The header begins `Ref Point 2 (machine zero)` + `Set Absolute` where f01 has `Ref Point 0 (current position)` (bytes `D8 10`, `E6 01` for `D8 12`), and then the same commands follow. Coordinates:

- The moves are at 226.866..266.866 / 242.224..282.224, i.e. the job sits at absolute machine position (221.666, 211.024) plus K = (5.2, 31.2). In f01 they sit at the origin plus K.
- Every header box (Process Top Left/Bottom Right, part bounds, element array, array) is the 10 um-grid-rounded box in the moves' frame MINUS K: 221.670..261.670 / 211.020..251.020. Document Min Point is the rounded box minimum in the moves' frame (226.870, 242.220); Document Max Point is the relative maximum, as in f01.
- So the rule in both modes is `header = grid10(box) - K`; only the header opening and the value of K's meaning differ. `RdJob::start = RdStart::Absolute` and `RdJob::pointer_offset` (K, file frame, default 0,0) encode it; `job_from_rd` recovers both and `check_reproduces("lightburn/f03.rd")` is byte for byte. The file runs correctly on the machine.
- Origin rule unchanged: `Document Min Point - Process Top Left` = (5.2, 31.2) again.

**What K is.** (5.2, 31.2) is the machine's **red-dot pointer offset**, configured in LightBurn's device settings (confirmed by the user; it is not the head's position, which was the earlier guess). The red dot is where the panel and the user position things; the beam is K away from it. LightBurn adds K to every laser move, in the file's (machine) frame, in both start modes:

- absolute (f03): moves = workspace position + K (workspace 221.666..261.666 / 211.024..251.024, moves 226.866..266.866 / 242.224..282.224);
- current (f01 ...): moves = K + (machine(design) - machine(origin)).

The header's boxes are `grid10(box in the moves' frame) - K` in both modes. Nlaser: `DeviceConfig::pointer_offset_x/_y`, lasers.conf `pointer_offset = X Y`, `--pointer-offset X,Y`, `RdJob::pointer_offset`. The sign is whatever the fixtures need: K is ADDED in the machine/file frame, after the design is mapped to machine axes (a top-right machine does not negate it). Tests build the f01 and f03 jobs from design coordinates with K = (5.2, 31.2) and reproduce the files byte for byte. The older fixtures (power test, box joint) have a different K, (-0.5, 32.3), which `job_from_rd` recovers like any other.

**Which operation applies K.**

| operation | K | evidence |
|---|---|---|
| send | applied (the beam follows the path) | LightBurn's behaviour, proven by f01..f07 and f03 |
| dryrun | applied (the same path, laser off) | follows from send |
| frame | NOT applied: the red dot traces the job box | the user's description ("Frame traces in the pointer frame"); NOT captured from LightBurn |
| goto, jog | NOT applied: the dot lands on the point | the user's description |
| pulse | NOT applied (a head-travel diagnostic) | our choice |

The bed check before send/dryrun (current mode) shifts the box by K; frame's does not. In absolute mode the encoder checks the written moves, K included, against the bed.

### The machine frame (top-right-origin Black Cat machine)

Design coordinates are X right, Y up, as in the DXF. LightBurn writes, for a job origin point `o` (in the design) and the pointer offset `h` = K (Start From = Current Position; `h` = 5.2, 31.2 in every fixture, the pointer offset -- see F03):

    file_x = h_x - (design_x - o_x)        file_y = h_y - (design_y - o_y)

That is, **both axes are negated**: X grows towards the LEFT of the design, Y grows DOWN it. It is exactly what a machine origin at the top-right corner predicts (machine X runs right to left, Y top to bottom), and there is no other offset: the only translation is "the origin point lands on the head".

Numbers, from the DXF in `tools/fixtures/lightburn_fixtures.lsp` (square (0,0)-(40,40), o = LightBurn's job origin):

| file | o (design) | design x 0..40 maps to file x | design y 0..40 maps to file y | process box (relative to origin) |
|---|---|---|---|---|
| f01 (top-left) | (0, 40) | 5.2 .. -34.8 | 71.2 .. 31.2 | x -40..0, y 0..40 |
| f06-BR (bottom-right) | (40, 0) | 45.2 .. 5.2 | 31.2 .. -8.8 | x 0..40, y -40..0 |
| f05 (centre) | (20, 20) | 25.2 .. -14.8 | 51.2 .. 11.2 | x -20..20, y -20..20 |
| f07 (top-left) | (0, 200) | circle at design (57,100) r 25: first Move at file (-44.366, 107.324) = design (49.566, 123.876), 25.000 mm from the centre | | x -292..0, y 0..200 |

In each, the cut coordinates equal `h - (design - o)` to the micrometre, and the process box equals `-(design box - o)`; `Document Min Point = h - (design extent) ` as already pinned by the origin rule.

The older fixtures (`lightburn_power_test*.rd`, `lightburn_box_joint.rd`) are the identity frame (machine origin bottom-left): X and Y keep their sign. The same design saved with job origin bottom-left and bottom-right (power test, bengal) has the same origin (-0.5, 32.3) and geometry TRANSLATED by -231.02 in X, not mirrored (file x -0.5..230.5 against -231.52..-0.5); the box joint (top-left) has box y -9.52..0, i.e. Y up. So all files fit

    rel_file = (sx * (design_x - o_x), sy * (design_y - o_y)),  sx = -1 for a machine origin on the right, sy = -1 for one on the top.

(That reading assumes the user's LightBurn job-origin labels for the old files, as recorded in tests/test_ruida_decode.cpp; the unit of evidence for the new files is the DXF itself.)

Consequence for naming: the decoder's `origin_corner` names a corner in FILE coordinates, which is the design corner only for a bottom-left machine. For a machine origin corner it must be mirrored in the axes the machine flips (f01 is design "top-left", f06-BR "bottom-right").

### Bed coordinates, and LightBurn's workspace (checked against f03)

Nlaser's convention for an absolute job: design coordinates are **bed coordinates**, (0,0) at the bed's bottom-left corner, X right, Y up. `x_m = W - x` on a machine that counts from the right, `y_m = H - y` on one that counts from the top (Black Cat: both, bed 900 x 600 as the controller reports).

What f03 says about LightBurn's workspace:

- `f03.lbrn2` has no device or bed element; it has `MirrorX="True" MirrorY="True"` on the project and one shape with `XForm -1 0 0 -1 261.66611 251.0244` over a 0..40 square. The square therefore occupies workspace x 221.666..261.666, y 211.024..251.024.
- `f03.rd`'s header box (Process Top Left/Bottom Right) is 221.670..261.670 / 211.020..251.020: the same numbers on LightBurn's 10 um grid. So **LightBurn's workspace coordinates are machine coordinates** (distances from the machine's own origin corner, top-right here), not bed coordinates. It matches our bed convention only on a bottom-left machine.
- In our convention that square is at bed x 638.334..678.334, y 348.976..388.976 (900 - 261.666 etc.; 600 - 251.024 etc.), and the encoder would write machine x 221.666..261.666, y 211.024..251.024 for it.
- The f03 moves are 226.866..266.866 / 242.224..282.224, i.e. the workspace position plus K = (5.2, 31.2). K is the pointer offset, so the RED DOT is at the workspace position and the beam at workspace + K. The encoder now adds K to the written moves (send, dryrun) and not to a frame; a hardware test of frame vs send on the same absolute job remains to be run.
- f03 is still reproduced byte for byte (the identity mapping on a bottom-left machine, where the file's frame is already positive).

## Raster (r01, r02, r03, r03-pt; 2026-11-17)

Files: `tests/fixtures/lightburn/r01.rd/.lbrn2` (Grayscale), `r02` (Dither, Jarvis), `r03` (Threshold), `r03-pt` (Threshold + Pass-Through), sources `R01-R02-wedge.png` (500x100 px, a 0..255 ramp left to right, 10 px/mm) and `R03-lines.png` (1 black, 3 white pixels repeating). All four: image 50 x 10 mm, layer 450.012 mm/s, power 40/50 (tube 2 10/20), `interval` 0.164948 mm, `dpi` 154, overscan 0, **`negative` 1**. Encoder and tests: `src/laser/ruida_raster.cpp`, `tests/test_ruida_raster.cpp`. **All four files are reproduced byte for byte** from the pixels and lines recovered out of the .rd itself (`raster_job_from_rd` -> `encode_rd_raster`), and r03 and r03-pt additionally from the PNG alone (`make_rd_raster`).

**Decoder.** All four decode with nothing unknown under magic 0x88. The magic guess used to pick 0xA3 (the commonest byte of a raster stream is not 0x00); `guess_magic` now trusts a header that decodes to `D8 10|11|12 F0` first. The raster commands (`C7`/`C2` Imd Power, `CA 41` Work Mode Part, `CA 01 05` Work Mode 6, `8B`/`89` line steps) were already in the table; no unknown command turned up in any of the four files.

**Frame.** Same rules as the vector files: `header box = grid10(visited box) - K`, `Document Min Point = grid10(box min)`, here with K = (3, 34) and `Ref Point 1 (anchor)` (`D8 11`) instead of `D8 12`; Work Mode Part is **2** (`CA 41 00 02`), colour #000000. The image lies at file x -47..3, y 34..44 (top-right machine: x grows towards the design's left, y towards its bottom). The box is that of the *burned* extent of the lines, not of the image (r02: -47.000..1.986).

**Stream.** After the header: a settings block SB (`Speed Axis`, `Work Mode 1`, `Layer Number`, `En Laser 2 Offset`, `Laser Device`, air, `Speed Laser`, `Min/Max Power 1`, `Min/Max Power 2`, `Laser On/Off Delay`, `Tube Start 1`: no `Layer End`, no Through Power, the delays AFTER the powers), a `Move Abs` to the first line's start, then **two** (pass block, SB) pairs (pass block = `Work Mode 6`, `Speed Axis`, `Layer End`, `Layer Number`, `En Laser 2 Offset`, `Laser Device`, `Tube Start 3`), then the lines, then `Block End`, `Array End`, `Block End` and the usual tail (cut counter = Manhattan cut length, floored mm). r01 and r03-pt put an extra `Work Mode 6` in front of each of the two later SBs; r02 and r03 do not (`lead_wm6`; tied to greyscale/pass-through in the CLI, not understood).

- **Lines.** One `Cut Rel X` / `Move Rel X` per run of equal pixels along a grid line; no overscan or lead-in (overscan 0). Lines alternate direction (boustrophedon: bidirectional on) and each ends where the next begins, so the step is a `Move Rel Y` (`8B`, one pitch), or, in a trimmed raster, a `Move Rel XY` (`89`) to the next line's first fired pixel. Line 0 runs +x in r02/r03 and -x in r01/r03-pt (not explained by the mode: r03-pt is threshold). First motion absolute, every later one relative (absolute above the relative limit, as in the vector files; no raster sample has a run that long).
- **Binary (threshold, dither, pass-through).** Each line is trimmed to its first and last fired pixel; gaps inside are `Move`s, fired runs `Cut`s. Blank lines are skipped (none in the samples: assumed). Fired pixels are the *light* ones (negative image). r03-pt: line 0 starts at x = 2.950 with `Cut -0.300` and its last pixel (black) is trimmed.
- **Greyscale (r01).** Nothing trimmed, nothing moved: every pixel is cut, a run of equal grey is one `Cut`, and before EVERY run (even if equal to the previous line's last) comes the power pair `C7 <p14>` + `C2 <p14>` (tube 1 and 2 immediate power). **p = float32(g) * float32(1/254) * 16383, + 0.5, floored, clamped** (g = grey level 0..255; light burns hard): exact for all 249 levels seen (65, 129, 194 ... 16254); not scaled by the layer's min/max, so the tube's full range is reachable whatever the layer says. The encoder applies `power_limit` to these values and refuses (the test lifts it to 100).
- **Pixel pitch** is 25.4/154 mm = 164.935 um (from the file's positions, not the stored `interval` 0.164948): 303 x 60 pixels (floor of 50/pitch and 10/pitch) in r02/r03, 302 columns in r01. Pass-through uses the image's own pixels: 500 x 100 at exactly 100 um, and the grid sits **half a pixel** (+50 um in x and y) in from the image edges: lines at x = -46.95 + 0.1 i, y = 44.05 - 0.1 k (pixel i is image column 499 - i; line k is image row 99 - k).
- **Grid phase and rounding.** Positions are micrometre integers of a regular grid (`floor(a + i * pitch)` fits y exactly in every file; x in r03/r02 only with a pitch a hair off 25400/154 and a fitted phase: the tie-breaking noise of LightBurn's float arithmetic; r01's x lines differ by 1 um between rows). The phase (where line 0 sits relative to the image edge) is **not derivable from the placement**: y offsets of -220 / -117 / +31 um in r03 / r02 / r01, and r01 and r02 share one placement. So `RdRasterGeometry` takes phase and pitch as inputs and `RdRaster::row_x_lines` carries r01's per-row grid. The r03 test passes the fitted numbers (pitch 164.93512, x phase 0.25456, y phase -220.1354 um).

**Image processing.** Sampling is nearest pixel at the centre of each grid cell (r03: zero differing pixels; area averaging from `resample` is not what LightBurn does). Threshold: fired when the (negative) level >= 128 (a guess; the samples are pure 0/255). **Not reproduced:** Jarvis (r02: Nlaser's `dither` differs from LightBurn's bitmap in about a third of the pixels in every orientation/serpentine variant tried; the r02 *stream* is reproduced from the bitmap extracted from the .rd); greyscale resampling (r01: nearest and linear fits leave 9-11 of 302 pixels wrong per line, so its pixel values come from the .rd). Vertical sampling is untested (the sources are constant down each column).

**CLI.** `ncad-laser rasterplan IMG --size W,H --dpi N --dither threshold|jarvis|grey ... --rd OUT.rd [--negative] [--passthrough] [--machine-origin tr] [--pointer-offset X,Y] [--ref-point 0|1|2]` writes a raster .rd with the grid on the image edges (no LightBurn phase, so it differs from LightBurn's file in those micrometres and, for pass-through, the half pixel). `.ljob` raster records are still refused by `plan`/`encode`.
