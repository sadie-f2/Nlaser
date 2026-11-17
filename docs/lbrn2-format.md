# LightBurn project files (.lbrn2): what we rely on

Source: the seven samples in tests/fixtures/lightburn (f01, f02-p2, f02-p3, f03, f05, f06-BR, f07; LightBurn 1.4.03, FormatVersion 1, saved on Windows: CRLF line ends, UTF-8, 4-space indent), each with its `.rd`. Code: include/ncad/laser/lbrn2.hpp, xml.hpp. Tests: tests/test_lbrn2.cpp.

Rule for this document: **"shown"** is in the samples; **"unknown"** is not, and the code either refuses it or says so. Everything unknown is listed at the end. Nothing here has been opened in LightBurn by us.

## Structure

One XML document, always in this order (identical in all seven):

    <LightBurnProject AppVersion="1.4.03" FormatVersion="1" MaterialHeight="0" MirrorX="True" MirrorY="True">
      <Thumbnail Source="BASE64 PNG"/>
      <VariableText> Start End Current Increment AutoAdvance, each <X Value=".."/> </VariableText>
      <UIPrefs> fourteen <Optimize_* Value=".."/> optimiser preferences, identical in every file </UIPrefs>
      <CutSetting type="Cut"> ... </CutSetting>       one per layer
      <Shape ...> ... </Shape>                         one per object
      <Notes ShowOnLoad="0" Notes=""/>
    </LightBurnProject>

Units: millimetres (the 40 mm DXF square is 40 in the file; f07's circles have Rx 1, 5, 25, 100 for diameters 2, 10, 50, 200). Every setting is an element `<name Value="..."/>`, never text.

`MirrorX="True" MirrorY="True"` is on every sample, all from the top-right-origin machine. Its meaning is not shown beyond that; we read it as "the machine's origin is on the right / on the top", warn when it disagrees with `--machine-origin`, and write it from the machine option.

## CutSetting (a layer)

Element order, from f01 (with `numPasses` after `PPI`, present only when Passes is not 1: f01 omits it, every other sample has it):

| element | sample value | we use |
|---|---|---|
| `index` | 2 | the layer's pen (LightBurn's layer index); shapes refer to it with `CutIndex` |
| `name` | Mark | kept in `Lbrn2Info::layer_names`, not in `LaserLayer` |
| `minPower`, `maxPower` | 15, 15 | tube 1 min/max power, percent. The .rd has Min/Max Power 1 15/15 |
| `minPower2`, `maxPower2` | 10, 20 | tube 2. The .rd has Min/Max Power 2 10/20: same numbers, so these are the tube 2 powers, kept separate from tube 1. `has_power2` is set when they differ from tube 1 |
| `speed` | 100 | mm/s (.rd: Speed Laser 1 100.000mm/s) |
| `PPI` | 0 | ignored |
| `numPasses` | 2, 3 | passes. f02-p2 -> 2 and f02-p3 -> 3, matching the .rd (two / three repeated blocks) |
| `perfLen` `perfSkip` `dotTime` `scanOpt` `overscan` `interval` `tabSize` `tabCount` `tabCountMax` `tabSpacing` | 0.01 0.01 1 byGroup 0 0.088 0.50038 1 1 50.038 | ignored (perforation, scan and tab settings); the writer emits these values |
| `priority` | 0 | `LaserLayer::order`; only 0 is shown, so what a non-zero priority does is unknown |

`type="Cut"` is the only value shown. Any other (Scan, Image, Tool...) is refused with a line number.

Layer colour: LightBurn's layer 2 is red in the .rd (`Layer Color Part 0, #FF0000`), i.e. ACI 1. That is the only index/colour pair the samples show, so `lbrn2_index_to_aci` maps 2 -> 1 and everything else to 7. The colour is not stored in the project (derived from the index); a job whose pen/colour pair is not that one therefore comes back from write+read with colour 7.

Air: no element in any sample, and the .rd has `Air Assist On` for all of them, so absent means on. How LightBurn spells "off" is unknown; the writer refuses air-off layers.

Layer enabled/"Output": unknown. The reader treats `<doOutput Value="0"/>` as `mode=skip` (our recollection of LightBurn's name, **unverified**); a project that uses another spelling would have its disabled layer cut. The writer refuses skip layers.

## Shapes

    <Shape Type="Path" CutIndex="2" VertID="0">
        <XForm>-1 0 0 -1 557.27362 301.70001</XForm>
        <VertList>V0 0c0x1c1x1V40 0c0x1c1x1V40 40c0x1c1x1V0 40c0x1c1x1</VertList>
        <PrimList>LineClosed</PrimList>
    </Shape>

    <Shape Type="Ellipse" CutIndex="2" Rx="1" Ry="1">
        <XForm>-1 0 0 -1 594.99902 354.89804</XForm>
    </Shape>

Shown: `Path` (f01, f02, f03, f05, f06-BR) and `Ellipse` (f07, four circles, Rx = Ry). No Rect, Group, Text, Bitmap or Polygon appears; they are refused by type and line.

- **XForm** is six numbers, assumed `a b c d e f` with `x' = a x + c y + e`, `y' = b x + d y + f`. The samples only have `-1 0 0 -1 tx ty`, which cannot tell the placement of `b` and `c` apart (unverified for rotations). The result is the point in **workspace coordinates** (below). Checked: f03's XForm 261.66611, 251.0244 puts the square at workspace 221.666..261.666 / 211.024..251.024, which is its .rd header box to the 10 um grid.
- **VertList** is `V<x> <y>` repeated, each vertex followed by `c0x<n>c1x<n>` (value 1) in every sample. Shown for line-only shapes, where these fields have no visible effect. For a Bézier the fields would be the control points `c0x c0y c1x c1y`; that is **unverified** and the reader accepts `c0x c0y c1x c1y` in that sense.
- **PrimList**: only `LineClosed` is shown: one closed polyline through the vertices in order, which becomes a closed `LaserPath` (so a square's four vertices are four points; no repeated end point). The reader also accepts explicit `L<a> <b>` (line) and `B<a> <b>` (cubic Bézier, vertex a's `c0` and vertex b's `c1` as controls, flattened to `--tol`, default 0.01 mm), consecutive primitives chaining into one path. Both spellings are from memory, **unverified**. The writer emits `L` chains for open paths, with the same caveat.
- **Ellipse** has no VertList. The reader flattens it to a closed polygon from the local point (Rx, 0) counter-clockwise, with the smallest power of two segments (minimum 8) whose sagitta is within the tolerance. f07's .rd has 32 / 64 / 128 / 256 segments per circle for D2 / D10 / D50 / D200, and this rule gives the same counts. LightBurn's vertices are not evenly spaced and its start points differ, so f07 does not reproduce byte for byte (see the acid test).
- `VertID`: 0 in f01, f02, f05, f06-BR; 1 in f03. Meaning unknown; we write 0, 1, 2... in shape order.
- Shapes within the file are in object order. Whether a Shape may reference a shared VertList by VertID without carrying its own is unknown.

## Where things are: workspace = machine coordinates

LightBurn's workspace coordinates are the machine's own coordinates (distance from the machine origin corner, top-right on these machines): f03's square is at workspace 221.666..261.666 / 211.024..251.024 and its .rd header box is the same numbers (see lightburn-fixtures.md, "Bed coordinates"). With the machine origin corner given:

- the reader maps workspace (xw, yw) to the design frame (X right, Y up). With the bed size known: bed coordinates, `x = W - xw` on a machine that counts from the right, `y = H - yw` from the top (`RdMachine::machine_to_bed`). Without a bed size: plain negation of the mirrored axes, so positions are only meaningful relative to each other, which is all Start From = Current Position uses;
- the writer applies the inverse and stores workspace coordinates directly in the VertList, with `XForm 1 0 0 1 0 0` (the samples' `-1 0 0 -1 tx ty` only records where LightBurn keeps each shape's origin).

Example (f01, no bed): XForm tx,ty = 557.27362, 301.70001 over a 0..40 square -> workspace x 517.27..557.27, y 261.70..301.70 -> design x -557.27..-517.27, y -301.70..-261.70.

## What is NOT in the file

- **Start From** (current / absolute / user origin), **Job Origin** (the nine dots), **device**, **bed size**, **origin corner**, **user origin**, red-dot offset: none appear. f01's job origin is "top-left" per notes.txt, and f03 is Absolute Coords, but the .lbrn2 files of f01 and f03 differ only in numPasses, VertID and the XForm translation. These are LightBurn UI/device state. Nlaser therefore takes them from options (`--start`, `--job-origin`, `--machine-origin`, `--bed`; pointer offset at encode) and records them in `Lbrn2Info` with `placement_from_file = false`. A job definition that must be self-contained will need them stored elsewhere than LightBurn's project, or a sample that shows where LightBurn keeps them (the `.lbdev` device file, perhaps; unknown).
- The XForm translation of the same square differs between f01, f02-p2, f05 and f06-BR (557.27/301.70, 403.84/317.18, 467.18/307.33, 468.59/315.78): the object was simply at a different place on the canvas each time. It is not a job origin.
- **Passes in f03**: lbrn2 says `numPasses 2` and f03.rd is 766 bytes with two repeated blocks; the earlier note "Passes 1" in lightburn-fixtures.md is wrong.

## Round trip and the acid test

`ncad-laser lbrn2ljob IN.lbrn2 -o OUT.ljob --machine-origin tr [--job-origin tl] [--start current|absolute --bed 900,600]`, then `plan`, then `encode --machine-origin tr --pointer-offset 5.2,31.2` (and the same `--start`/`--bed`):

| sample | options | result vs the .rd |
|---|---|---|
| f01 | origin tl | byte for byte |
| f02-p2, f02-p3 | tl | byte for byte |
| f05 | origin c | byte for byte |
| f06-BR | origin br | byte for byte |
| f03 | tl, absolute, bed 900x600 | byte for byte (the reader puts the planner's head at machine zero, the bed's top-right, in absolute mode; LightBurn's file starts at the corner nearest that point) |
| f07 | tl | same layer block, 960 cuts, 4 moves; differs in start points and circle order, in vertex spacing (LightBurn's are slightly irregular: cut length 1646.31 mm against our 1646.09 mm) and so in the move lengths (260.2 against 221.3 mm) and the file sum |

Tests pin the six byte-equal cases (test_lbrn2.cpp) and f07's structure. Precision: `.ljob` holds micrometres, the .lbrn2 five decimals; the .rd encoder works on the ljob values, which is why these still match to the byte.

`write_lbrn2` copies the samples' element order, attributes, `VariableText`, `UIPrefs` and `CutSetting` values, writes CRLF, and omits only the drawn thumbnail (it writes a blank 1x1 PNG; whether LightBurn needs one is unknown). **It has not been opened in LightBurn.** What is checked is: write then read gives the same job (every machine corner, with and without a bed; tests), and the element/attribute sequence against f01 and f02-p2.

## Unknown, in short

Rect/Group/Text/Bitmap and any non-Path/Ellipse shape; Scan, Image and Tool layer types; the XForm placement of `b`, `c` (rotation/shear); Bézier and explicit-primitive spelling (`B`/`L`, `c0x..`); open-path spelling; layer output on/off and air-off elements; non-zero `priority` semantics; `VertID` semantics; `MirrorX/Y` semantics on other machines; where LightBurn keeps Start From, Job Origin and the device; LightBurn's colour for any layer index but 2; whether the Thumbnail is required; FormatVersion other than 1.

## Layer colours for LBRN2IN (uncertain)

The project does not store a layer's colour; LightBurn derives it from the layer index. Only index 2 = red (#FF0000) is evidenced (the samples' .rd). `lbrn2_layer_rgb` carries the palette C00..C29 **from memory of LightBurn, unverified beyond C02**:

    00 000000  01 0000FF  02 FF0000  03 00E000  04 D0D000  05 FF8000  06 00E0E0  07 FF00FF  08 B4B4B4  09 0000A0
    10 A00000  11 00A000  12 A0A000  13 C08000  14 00A0FF  15 A000A0  16 808080  17 7D87B9  18 BB7784  19 4A6FE3
    20 D33F6A  21 8CD78C  22 F0B98D  23 F6C4E1  24 FA9ED4  25 500A78  26 B45A00  27 004754  28 86FA88  29 FFDB66

`lbrn2_assign_aci` gives each layer the nearest still-unused ACI (squared RGB distance, ACI palette of `aci_rgb`), so two LightBurn layers never share a pen; an index outside the table starts from ACI 7. Where the recalled colour is wrong the only effect is a different, still distinct, ACI colour; the cut settings are unaffected. The older `lbrn2_index_to_aci` (2 -> 1, else 7) is kept for the .ljob reader.

LBRN2IN (src/ncad/lbrn2in_command.cpp) uses this for layer colours and for the pen table it leaves in the LASER session (pen = the ACI). `LASER Pens Load x.lbrn2` and `LASERPENS` pointing at a .lbrn2 build the same table. Writing a drawing with `LASER Save x.lbrn2` uses the pen number as the layer index, so its LightBurn colour follows the table above, not the drawing's ACI colour.
