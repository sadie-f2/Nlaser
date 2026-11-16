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

- Layout: these files use a NEWER layer layout than the older fixtures (extra `Speed Axis 15mm/s` before each `Layer End`; after the first Move a `Work Mode 6` / `Tube Start 3` block). The encoder does not reproduce f01 byte for byte yet, so no `check_reproduces` for them; decoding is clean (0 unknown, 0 bad length, file sum ok).
- Passes (f01 vs f02-p2 vs f02-p3): each extra pass repeats the block `Work Mode 6 ... En Laser Tube Start 1` plus the cuts; the Move is written once. File size 625 / 764 / 903 bytes (+139 per pass), cut counter `06 20` 160 / 320 / 480 mm, cuts 4 / 8 / 12. Encoder refusal for passes != 1 is left in place until the new layout is implemented.
- f05, f06-BR and f07 were saved with Passes still at 2 (8 cuts, 764 bytes; f07 traces each circle twice in one polyline).
- Centre origin (f05): process box is -20..20 both axes; `origin = Document Min Point - Process Top Left` still holds (5.2, 31.2 in every file: Ref Point 0, current position); summarize_rd reports "other".
- Corner names: the decoder names corners in file coordinates, 180 degrees from LightBurn's label. f01 (LightBurn top-left) is box x -W..0, y 0..H, reported "bottom-right"; f06-BR (LightBurn bottom-right) is box x 0..W, y -H..0, reported "top-left". Same rule, mirrored in both axes.
- Arcs (f07): per pass 32 / 64 / 128 / 256 segments for D2 / D10 / D50 / D200, i.e. the smallest power of two with sagitta R(1-cos(pi/n)) <= tolerance; measured max sagitta 0.0082 mm. Tolerance lies in [0.0075, 0.0192) mm, probably 0.01 mm. Match this in the planner / dxf2ljob default later (not changed now). Vertices are not evenly spaced (chord varies about 7%).

### The machine frame (top-right-origin Black Cat machine)

Design coordinates are X right, Y up, as in the DXF. LightBurn writes, for a job origin point `o` (in the design) and the head's position `h` when the file was saved (Start From = Current Position; `h` = 5.2, 31.2 in every fixture):

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
