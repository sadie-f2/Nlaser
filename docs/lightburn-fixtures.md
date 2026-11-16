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
