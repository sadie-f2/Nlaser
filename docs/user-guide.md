# Nlaser user guide

For people who draw in NotoCAD and cut on a Ruida controller (Black Cat
lasers, RDC6442S), and are used to LightBurn. Every example output below was
produced by running the tools, against `nlaser_fakectl` (the test suite's fake
controller on a loopback port) where a device was needed. A fake controller
does not move, so head positions in those transcripts are not what a real
machine would report.

Contents

1. What Nlaser is
2. Setup
3. Connecting
4. The LASER command
5. Placement
6. `ncad-laser` reference
7. Imports
8. Safety
9. Troubleshooting
10. Status: what has been tried on hardware

---

## 1. What Nlaser is

Nlaser turns geometry into a Ruida job and sends it to the controller. It has
two front ends over one core:

- **`LASER`, a command inside NotoCAD** (`ncad` and `ncad_gui`, NotoCAD branch
  `laser`). It cuts what is in the drawing: pick pens, set the job origin,
  build the job, frame it, send it. It also adds `SVGIN`, `PDFIN` and
  `LBRN2IN` to import other formats.
- **`ncad-laser`, a command-line tool.** It works on files (`.ljob`, `.rd`,
  `.lbrn2`, SVG, PDF) and talks to a controller directly. It is also the
  tool to reach for when something is wrong, because every step is a separate
  command.

Relation to LightBurn:

- The `.rd` files Nlaser writes are meant to be the same bytes LightBurn
  writes. The encoder reproduces LightBurn's own files byte for byte for the
  samples in `tests/fixtures/lightburn/` (square, 2 and 3 passes, absolute
  start, centre origin, bottom-right origin). `ncad-laser rd2txt` decodes any
  `.rd`, LightBurn's included.
- LightBurn `.lbrn2` projects can be read (`LBRN2IN`, `lbrn2ljob`,
  `Pens Load x.lbrn2`) and written (`LASER Save x.lbrn2`, `ljob2lbrn`). A
  written `.lbrn2` has not yet been opened in LightBurn
  (`docs/lbrn2-format.md`), and only a subset of LightBurn's file is
  understood: paths and ellipses on Cut layers.
- Nothing in Nlaser needs LightBurn. Only one program at a time can talk to
  the controller over Ethernet, so close LightBurn first (section 9).

Nlaser's own job format is `.ljob`, a small text file (paths, layers, origin).
`ncad-laser plan` orders the paths, `encode` turns a planned `.ljob` into
`.rd`. The LASER command does both internally.

**Units: millimetres or inches.** A job is always in mm; the drawing may be
in either. NotoCAD has no drawing units, so you say which: the LASER command
reads the setting `LASERUNITS` (`mm`, the default, or `in`), and `dxf2ljob`
takes `--units mm|in`, otherwise reads the DXF's `$INSUNITS` (1 = inches,
4 = mm). With inches, every coordinate, the job origin and widths are scaled by
25.4 as the job is built; `LASERTOL` stays in mm. A DXF with no `$INSUNITS` is
taken as mm, with a warning; one with any other value is refused until you
give `--units`. The job summary always states the units used. Speeds are mm/s,
power is percent, the bed and the Goto/Step values are mm. Imports convert
their own units (SVG `in`, PDF 1/72 in) to mm.

## 2. Setup

### 2.1 Building

Standalone (the tool and the tests; no NotoCAD needed):

    cmake -S . -B build -G Ninja
    cmake --build build
    ./build/tests/nlaser_tests

With NotoCAD (adds `dxf2ljob` to `ncad-laser`), NotoCAD's source tree beside
this one or named explicitly:

    cmake -S . -B build -G Ninja -DNLASER_WITH_NOTOCAD=ON [-DNLASER_NOTOCAD_DIR=../NotoCAD]
    cmake --build build

The LASER command lives in NotoCAD's build. NotoCAD's CMake finds the Nlaser
tree beside it (`NCAD_LASER_DIR`, default `../Nlaser`) and builds `ncad` and
`ncad_gui` with LASER, SVGIN, PDFIN and LBRN2IN:

    cmake -S . -B build -G Ninja -DNCAD_LASER_DIR=/path/to/Nlaser
    cmake --build build        # ./build/src/app/ncad

Needs CMake 3.25. PDF and Illustrator import need qpdf at build time
(`-DNLASER_WITH_QPDF=AUTO|ON|OFF`; AUTO uses it when found). Without qpdf,
`PDFIN` says so and `pdf2ljob` refuses; SVG import is unaffected. Network
support (`NLASER_WITH_NET`, default ON) is what connects to a controller;
without it LASER can build and save jobs but not send.

### 2.2 lasers.conf

Devices are named in `$XDG_CONFIG_HOME/ncad/lasers.conf`, else
`~/.config/ncad/lasers.conf` (`--config PATH` overrides for `ncad-laser`).
INI: one `[name]` section per device, `#` or `;` start a comment. Unknown keys
and malformed values are errors that name the file and line, on purpose: a
typo that fell back to a default `max_power` is how a laser runs hotter than
its owner meant.

A Black Cat on USB:

    [blackcat]
    serial = /dev/cu.usbserial-A904TTIM   # the controller's side USB-B port
    origin_corner = tr                    # Black Cat machine origin: top-right
    bed = 900 600                         # as the controller reports it
    max_power = 30                        # Send refuses any layer above this
    pointer_offset = 5.2 31.2             # red-dot offset (LightBurn device setting)

Every key:

| Key | Default | Meaning |
| --- | --- | --- |
| `driver` | `ruida` | The only driver. |
| `host` | | Controller IP, over Ethernet (UDP). Give `host` or `serial`, not both. |
| `serial` | | USB serial device, e.g. `/dev/cu.usbserial-XXXX`. |
| `baud`, `hw_flow` | 115200, true | Serial only. USB bridges ignore the baud rate. `hw_flow = false` is `--no-flow`. |
| `magic` | `0x88` | The swizzle byte. `auto` is not implemented. |
| `port` | 50200 | Controller UDP port. |
| `local_port` | 40200 | Our UDP port; the controller answers there. |
| `keypad_port` | 50207 | Panel-key channel, for `pulse --method key`. |
| `max_packet` | 1000 | Largest packet payload, bytes. |
| `max_power` | 100 | Operator power cap, percent. See 2.3. |
| `power_limit` | 80 | Hard power limit, percent. File-only. See 2.3. |
| `ack_timeout_ms` | 2000 | Wait for each acknowledgement. |
| `bed` | unset | `W H` in mm. Used to map bed coordinates on machines that count from the right or top, and to check that a job fits. If unset the tool asks the controller (it may not answer over serial). |
| `origin_corner` | unset | `bl br tl tr`: which corner the machine counts from. Black Cat: `tr`. |
| `flip_x`, `flip_y` | false | Extra axis mirrors, applied the same way. |
| `pointer_offset` | `0 0` | `X Y` mm, the red-dot pointer's offset from the beam, in the machine frame, as in LightBurn's device settings. See section 5. |

`ncad-laser devices` lists what is configured:

    blackcat  127.0.0.1  port 50299, magic 0x88, power_limit 80%, bed 900 x 600 mm, pointer offset 5.2, 31.2 mm

With exactly one device in the file, commands that need one pick it without
`--device`. With several, you must say which: guessing which laser to fire is
not acceptable.

### 2.3 Two power limits

| Setting | Meaning | Default | Changed by |
| --- | --- | --- | --- |
| `max_power` | The operator's cap for a material or a day | 100 | lasers.conf, or `--max-power` on `send` |
| `power_limit` | Hard limit for machine protection (tube maintenance) | 80 | lasers.conf only |

Both are applied by the encoder, so `send`, `dryrun`, `encode` and the LASER
command's Send and Save `.rd` all obey them. The effective cap is the smaller
of the two, and a refusal says which stopped the job:

    ncad-laser: layer 5 asks for 70.0% power; operator max_power 60% allows no more
    ncad-laser: layer 0: power above the hard limit of 80% (maintenance setting, power_limit in lasers.conf)

`power_limit` holds even with no lasers.conf and for `ncad-laser encode`, which
writes a stick file with no device at all. It has no flag and appears in no
prompt, on purpose: a flag or prompt is one keystroke from being raised by
habit, a script, or a recalled history line. Raising it means editing the
file, a deliberate act by whoever owns the machine. Frames and dry runs write
power 0 and are not held to it. The panel's Pulse key cannot be checked
against either limit, so `pulse --method key` is refused unless both are 100.

### 2.4 Pen tables (`.lpt`)

A pen is an AutoCAD colour index (ACI, 1-255). An entity's pen is its
effective colour (so a BYLAYER entity takes its layer's colour). The pen table
says how each pen cuts. An `.lpt` is `layer` lines in `.ljob` syntax; `#`
starts a comment. `examples/test.lpt`:

    layer 1 color=1 speed=100 power=15,15 order=2   # red: cut, last
    layer 3 color=3 speed=150 power=15,15 order=1   # green: score first
    layer 5 color=5 speed=100 power=15,15 order=2   # blue
    layer 7 color=7 speed=100 power=15,15 order=2   # white/black

Fields after the pen number (all optional):

| Field | Values | Meaning |
| --- | --- | --- |
| `color=N` | ACI | Colour recorded in the file (defaults to the pen). |
| `mode=` | `auto` `vector` `raster` `skip` | Default `auto`. `skip` leaves the pen out. |
| `speed=` | mm/s | |
| `power=MIN,MAX` | percent | Tube 1 minimum and maximum power. |
| `power2=MIN,MAX` | percent | Tube 2, if different. |
| `passes=` | 1-99 | |
| `air=` | `on` `off` | |
| `order=` | integer | Lower numbers are cut first (score before cut). |

**A pen the table does not list runs at power 0**, and the job report says so
(`warning: pens 5, 7 have no pen-table entry and will run at power 0`). That is
the safe default: a forgotten pen draws a path with the laser off rather
than cutting at someone else's settings.

`LASERPENS` (a system variable) holds the table's path; the LASER `Pens` mode
loads, edits and saves it. `Pens Load` also takes a LightBurn `.lbrn2`,
using its layers' cut settings.

## 3. Connecting

Three ways in. Use USB first.

**USB (serial).** The controller has a USB-B socket on its side; the USB-A
socket on top is the stick port and is not this. Connect a data cable from the
Mac to the side port; it appears as `/dev/cu.usbserial-*` (Linux:
`/dev/ttyUSB0`). Put it in lasers.conf as `serial =`, or give `--serial DEV`.

    ls /dev/cu.usbserial-*

Serial has no checksum and no acknowledgements in this framing (it follows
MeerK40t's reverse-engineered USB work), so **a send over serial is not
confirmed by the controller**. A command is written and assumed received.
Reads (status, head position) may also go unanswered; the tool then warns
instead of checking, and says so.

**Ethernet (UDP).** Give the controller a static address (set on the controller),
put a computer on the same subnet, and use `host =` or `--host IP`. The
controller listens on UDP 50200 and answers on 40200, so **only one program
per computer can talk to it: quit LightBurn first.** Nlaser checks each
packet (ACK/NAK, resend on NAK).

**USB stick.** No connection at all: write an `.rd` and copy it to the stick.

    ncad-laser encode planned.ljob -o card.rd --machine-origin tr --pointer-offset 5.2,31.2

or LASER `Save` with a `.rd` name. The device named in lasers.conf supplies
the origin corner, flips and magic when you use LASER; with `encode` you give
`--machine-origin`, `--bed` and `--pointer-offset` yourself. The hard power
limit applies to the file.

First contact is read-only: `ncad-laser counters --device blackcat` and
`status` only read.

## 4. The LASER command

`LASER` is **modal**, in the style of R12's `LAYER` and `PEDIT`: it keeps
prompting until you press Enter at the `Laser` prompt or type `Quit`. Every
keyword does its work and returns to the prompt. A keyword that fails prints
`error: ...` and returns to the prompt; the command is not ended. (So a script
using `(command "LASER" ...)` cannot tell that a step failed; this is accepted,
see `docs/design-notes.md`.) Output between prompts is the first line(s) of
the next prompt.

Keywords can be abbreviated to the capitals in the prompt: `DE`, `PE`,
`ORI`, `ORD`, `J`, `M`, `PR`, `F`, `DR`, `SA`, `SE`, `PU`, `STA`, `STO`, `C`,
`Q`. Where two words share a start, the prompt capitalises enough to tell them
apart (Device/DRyrun, Status/Stop, Pens/Preview/Pulse, Save/Send).

Leaving:

- **Enter** at the `Laser` prompt, or **Quit**, ends the command. If you edited
  the pen table and did not save it, it says
  `Pen table edits were not saved (Pens Save)`: that is the one thing that
  dies with the mode.
- **Esc** ends the whole command from anywhere. What was already done stays
  done: a job built, a file written, a jog sent. Nothing sent to the
  controller is undone by Esc; use `Stop`.
- In a sub-mode (Move, Pens, Order) **Enter** returns to the `Laser` prompt.

Prompt tree:

    Laser [DEvice/PEns/ORIgin/ORDer/Job/Move/PReview/Frame/DRyrun/SAve/SEnd/PUlse/STAtus/Counters/STOp/Quit] <Quit>
    |
    +-- Device       name a [section] of lasers.conf ("Device (blackcat):"); Enter keeps the current one
    +-- Pens         [LIst/LOad/SEt/SAve] <done>
    |     LIst         show the pen table
    |     LOad         file: .lpt or .lbrn2, "." for none
    |     SEt          pen, speed, power, passes, air; Enter keeps each value
    |     SAve         write the table as .lpt
    +-- ORIgin       shows the job origin; asks Start from [Current/Absolute]
    +-- ORDer        [List/Pick/Start/Clear] <done>
    |     List         the cutting order
    |     Pick         pick entities in the order to cut them; Enter ends
    |     Start        a point on a closed path to start it there; Reverse [Yes/No]
    |     Clear        drop every override
    +-- Job          Area [Extents/Window/Display/Limits] <Extents>; builds and plans the job
    +-- Move         [Goto/+X/-X/+Y/-Y/Fine/Coarse/Step] <done>
    |     Goto         pick a point
    |     +X -X +Y -Y jog by the step
    |     Fine         step 0.1 mm
    |     Coarse       step 10 mm
    |     Step         any step, mm (default 1)
    +-- PReview      text listing of the planned order (not a picture)
    +-- Frame        trace the job's box with the laser off
    +-- DRyrun       trace the whole path with the laser off
    +-- SAve         .ljob, .rd or .lbrn2, by the extension (no extension: .ljob)
    +-- SEnd         shows the job, asks Yes/No (default No), then uploads and starts it
    +-- PUlse        FIRES THE LASER: power, duration, asks Yes/No (default No)
    +-- STAtus       state, head position, bed, file progress
    +-- Counters     card id, firmware, times, job count
    +-- STOp         stop command
    +-- Quit

Notes on each:

- **Device** chooses the laser for everything after it (stored in `LASERDEV`,
  not in the drawing). With one device in lasers.conf you can skip it.
- **Pens** edits a working copy; nothing changes on disk until `Pens Save`.
  `Set` asks Pen number (1-255), Speed, Power, Passes (1-99), Air; `Power`
  sets the maximum and lowers the minimum if it was higher. A pen not in the
  table is created at power 0 until you give it a power. If you ran `LBRN2IN`,
  its pens are the table until you `Pens Load` something else.
- **Origin** is described in section 5.
- **Order** is the designer's say over the planner. Picked paths are cut first
  in your order; the rest are optimised around them. A new `Job` clears the
  overrides (path ids are renumbered).
- **Job** reads the drawing (Extents: everything on visible layers; Window:
  two corners; Display; Limits), applies the pen table, joins and orders the
  paths, and reports pens, bounds, cut and travel length, estimated time, and
  warnings. Curves are flattened to `LASERTOL` (default 0.01 mm). Entities on
  off or frozen layers are not cut and are counted. HATCH, IMAGE and other
  entities with no class are skipped with a warning; wide polylines cut as
  centre lines.
- **Frame** sends only the bounding box path, with no cut command in it. The
  red dot traces the box (the pointer offset is not applied).
- **DRyrun** sends the whole path as moves at power 0 (no cut command in the
  file). The beam path, with the pointer offset applied, is what you watch.
- **Save** to `.rd` uses the chosen device's origin corner and applies the same
  power ceiling as Send; with no device it assumes a bottom-left machine and
  says so. To `.lbrn2` it writes a LightBurn project (the pen number is the
  layer index; the colour LightBurn shows follows its own palette, not the
  drawing's). An existing file asks `Overwrite? [Yes/No] <No>`.
- **Send** runs the same checks first (power limits, bed fit, placement), so a
  job that would be refused is refused before you are asked to confirm it.
  `LASERCONFIRM` = 0 skips the question; leave it at 1.
- **Pulse** and **Stop** are in section 8.

### A worked session

Draw a 30 mm square (red, pen 1) and a 10 mm circle (blue, pen 5), save a UCS
named `LASER` at the square's corner, and cut it with Start From Absolute.
(Output trimmed; every `Laser [...]` prompt is shortened to `Laser [...]`.
Against the fake controller.)

    Command: UCS
    ... O ... 100,100
    Command: UCS
    ... S ... LASER
    Command: UCS ... W
    Command: LASER
    Laser [...] <Quit>: DE
    Device (blackcat): blackcat
    Device blackcat at 127.0.0.1
    Laser [...] <Quit>: PE
    Pens [LIst/LOad/SEt/SAve] <done>: LO
    Pen table file (.lpt or .lbrn2, . for none): card.lpt
    Pen table card.lpt (4 pens)
    Pens [LIst/LOad/SEt/SAve] <done>: SE
    Pen number (1-255) <done>: 5
    Speed mm/s <100>: 80
    Power % <15>: 12
    Passes <1>:
    Air assist [ON/OFf] <On>: ON
    pen 5: 80 mm/s, power 12-12%, passes 1, air on
    Pens [LIst/LOad/SEt/SAve] <done>:
    Laser [...] <Quit>: ORI
    Job origin 100.000,100.000 (UCS LASER)
    Start from [Current/Absolute] <Current>: A
    Job origin 100.000,100.000 (UCS LASER); start from absolute machine coordinates
    Laser [...] <Quit>: J
    Area [Extents/Window/Display/Limits] <Extents>:
    Job from Extents: 2 paths on 2 pens, 2 entities
      layer 1: auto, 100 mm/s, power 15-15%, passes 1
      layer 5: auto, 80 mm/s, power 12-12%, passes 1
    Bounds 100.000,100.000 .. 130.000,130.000 mm (30.000 x 30.000)
    Origin 100.000,100.000 (UCS LASER)
    Cut 151.4 mm, travel 16.2 mm, estimated 2.9 s
    Laser [...] <Quit>: PR
    Cutting order: 2 paths
        1  pen 1  closed  4 pts  100.000,100.000 -> 100.000,100.000  120.00 mm  id 0
        2  pen 5  closed  50 pts  111.355,111.577 -> 111.355,111.577  31.40 mm  id 1
    Laser [...] <Quit>: M
    Move [Goto/+X/-X/+Y/-Y/Fine/Coarse/Step] <done> (step 1 mm): G
    Go to point: 115,115
    moved to 115.000,115.000; head at machine 785.000,485.000 (bed 115.000,115.000)
    Move [...] (step 1 mm): <Enter>
    Laser [...] <Quit>: F
    Framing the job's bounding box on blackcat (laser off)
    Laser [...] <Quit>: DR
    Tracing the whole planned path on blackcat with the laser off (no cut command, power 0)
    Laser [...] <Quit>: SA
    Save job as (.ljob, .rd or .lbrn2): card_abs.rd
    Saved card_abs.rd (1141 bytes)
    Laser [...] <Quit>: SE
    Job (unnamed) on blackcat
      layer 1: 100 mm/s, power 15-15%, passes 1, air on
      layer 5: 80 mm/s, power 12-12%, passes 1, air on
      bounds 100.000,100.000 .. 130.000,130.000 mm (30.000 x 30.000)
      origin 100.000,100.000, absolute machine coordinates; 1141 bytes
    Cut 151.4 mm, estimated 2.9 s.
    Run it on blackcat now? [Yes/No] <No>: Y
    Sent 1141 bytes to blackcat
    Laser [...] <Quit>: <Enter>
    Pen table edits were not saved (Pens Save)

Two things to read in that transcript: the pen 5 edit was not saved to
`card.lpt` (the mode says so on the way out), and the `Goto 115,115` landed
at bed (115, 115), which is machine (785, 485) on a top-right 900 x 600 bed.
`(unnamed)` is because the drawing had no name yet.

## 5. Placement

Where the job lands is the part LightBurn users trip over. There are three
separate things: the **start mode**, the **job origin**, and the **red-dot
offset**.

### 5.1 Start From: Current or Absolute

Set with LASER `Origin` (`Start from [Current/Absolute]`), `--start` on
`encode`, or `start=absolute` in a `.ljob` header. Default: Current.

- **Current** (LightBurn's "Current Position"): the job's origin goes where
  the head is when the job starts. Put the head (red dot) where the
  origin should be, then Send. Before `send`, `frame` and `dryrun`, the tool
  reads the head position and the bed size and refuses a job whose box would
  leave the bed, with the numbers:

      ncad-laser: the job would leave the bed: with the head at machine 0.000, 0.000 mm it covers x -24.800..5.200, y 1.200..31.200, and the bed is 0..900.000 by 0..600.000. Move the head (ncad-laser goto/jog) or use start=absolute

  If the head position or bed cannot be read (serial need not answer), it
  **warns** and goes on. The warning is printed, never silent.
- **Absolute**: the drawing's coordinates ARE bed coordinates: (0,0) at the
  bed's bottom-left, X right, Y up, mm. On a machine that counts from the right
  or top (Black Cat: both), the tool maps them with the bed size
  (`x_machine = W - x`, `y_machine = H - y`). A machine that does not count
  from the bottom-left needs the bed size (lasers.conf `bed`, `--bed W,H`, or
  the controller's own report), and any coordinate outside the bed is refused:

      ncad-laser: absolute start: the job leaves the bed (machine x 875.200..905.200, y 601.200..631.200 mm; bed 900.000 x 600.000 mm)

  (A job drawn at 0..30 leaves the bed in absolute mode once the pointer offset
  is added; that is the check working.)

LightBurn's workspace coordinates are *machine* coordinates (distance from
the machine's own origin corner), not bed coordinates; the two agree only on a
bottom-left machine. See `docs/lightburn-fixtures.md`.

### 5.2 The job origin: UCS `LASER`

The job origin is the origin of the UCS named `LASER`, else the world origin
(0,0). It is a fixed point in the drawing, never a corner of the selection.
`Origin` shows it but does not set it: define it with `UCS` (Save, name
`LASER`). In Current mode that point is what lands on the head. Only the
origin is used; a rotated `LASER` UCS is not applied yet (`Origin` says so).
`dxf2ljob --origin X,Y` overrides it for that tool; for `encode`,
`--origin x,y` or a corner of the job's box (`bl br tl tr`) does.

### 5.3 The red-dot pointer offset

Your lasers' red dot is not where the beam is: the controller positions the
dot, and the beam is `pointer_offset` away (5.2, 31.2 mm on the machines
documented here, the same K LightBurn has in its device settings). Nlaser
adds that offset where the **beam** must be right and leaves it out where the
**dot** is what you are looking at:

| Operation | Offset applied | Why |
| --- | --- | --- |
| `send` | yes | the beam follows the path (LightBurn's behaviour, proven by its files) |
| `dryrun` | yes | the same path, laser off |
| `encode`, `Save .rd` | yes, if given | it is the file that would be sent |
| `frame` | no | the red dot traces the box (from your description of LightBurn; not captured from LightBurn) |
| `goto`, `jog`, `Move` | no | the dot lands on the point |
| `pulse` | no | a head-travel diagnostic |

Net effect: Frame shows you, with the dot, the box the job occupies; Send puts
the beam inside it. In `encode` pass `--pointer-offset 5.2,31.2` or the stick
file will be offset wrongly; `lbrn2ljob` + `encode` with it reproduces
LightBurn's own `.rd` for the samples.

### 5.4 Goto means two things

`Move` > `Goto` picks a point **in the drawing**. What that means depends on
Start From. All Move operations are pointer moves: laser off, no offset.

- **Absolute:** the drawing is the bed, so the point is a bed coordinate. The
  head goes where an absolute job would cut that point.
- **Current:** the job origin is wherever the head was. The head position is
  read once, on the first Move operation after entering Move, and kept as an
  anchor. Goto then moves to anchor + (point - job origin), mapped through the
  machine's axis directions (on a top-right machine, drawing +X is machine
  minus). The anchor is never re-read: repeated Gotos do not drift, jogs do not
  change it, and leaving and re-entering Move takes a new one. If the head
  position cannot be read (a serial link without replies) Goto is refused with
  a message; jog still works.

Jog (`+X -X +Y -Y`) is always a displacement from the head in **bed**
directions (`+X` is rightwards on the bed whatever the machine's origin), and
refuses a target outside the bed:

    Move [...] (step 0.1 mm): +X
    error: the target leaves the bed: machine -0.100, 0.000 mm; the bed is 0..900.000 by 0..600.000

(Here the fake head was at machine 0, the top-right corner, so "right" is out
of the bed.)

`ncad-laser goto` is the separate tool: its X Y are bed coordinates, or raw
machine coordinates with `--machine`.

## 6. `ncad-laser` reference

    ncad-laser SUBCOMMAND ... 

`WHERE` below means one of `--host IP`, `--serial DEV`, `--device NAME`.
`ncad-laser` with no arguments prints usage.

### Device options (counters, status, watch, settings, stop, goto, jog, frame, dryrun, raw, pulse, send)

| Option | Meaning |
| --- | --- |
| `--device NAME` | `[NAME]` in lasers.conf; may be omitted when the file has exactly one device and no `--host`/`--serial` |
| `--config PATH` | lasers.conf to read (an explicit path must exist) |
| `--host IP` / `--serial DEV` | exactly one of the two |
| `--baud N`, `--no-flow` | serial speed (default 115200) / no RTS/CTS |
| `--magic N` | swizzle magic (default 0x88) |
| `--port N`, `--local-port N` | UDP ports (50200 / 40200) |
| `--chunk N` | largest packet payload (default 1000) |
| `--timeout MS` | acknowledgement wait (default 2000) |
| `--pointer-offset X,Y` | red-dot offset, mm (see 5.3) |
| `--bed W,H` | bed size, mm (lasers.conf `bed`); goto, send, frame and dryrun check against it |
| `--yes` | `pulse` and `send`: do not ask for `yes` first |
| `--max-power PCT` | `send` refuses any layer above this |
| `--keypad-port N` | panel-key channel for `pulse --method key` |

Flags override the same setting from the device's section.

### File commands

**`dxf2ljob IN.dxf -o OUT.ljob [--pens FILE.lpt] [--tol MM] [--origin X,Y] [--units mm|in] [--flatten ncad|lightburn]`**
(only when built with NotoCAD). One path per polyline NotoCAD draws, the pen
being its ACI colour. Pens come from the table; unlisted pens get power 0 with
a warning. The origin is the drawing's `LASER` UCS, else 0,0 (`--origin` is in
drawing units). `--units` says what a drawing unit is; without it `$INSUNITS`
decides (see Units above). Wide polylines and SOLIDs cut as outlines,
with a warning. The output is raw: `plan` it next.

`--flatten lightburn` makes circles, arcs and ellipses the same polygons
LightBurn would (Bezier quadrants, power-of-two step counts, a full circle
starting at 180 degrees and running clockwise) instead of NotoCAD's uniform
angle steps. Use it to compare a job byte for byte with LightBurn's; the
default `ncad` is fine for cutting. Checked against LightBurn's f07 circles;
arcs and ellipses use the same construction but have no fixture.

    $ ncad-laser dxf2ljob hershey_N.dxf -o h.ljob --pens examples/test.lpt
    ncad-laser: warning: pens 4, 8 have no pen-table entry and will run at power 0
    h.ljob: 6 paths on 2 pens from 6 entities (0 on off or frozen layers not cut); drawing units mm
    origin 0.000,0.000 (world); raw: plan it next

**`svg2ljob IN.svg -o OUT.ljob [--pens FILE.lpt] [--px-per-in N] [--tol MM]`**
No NotoCAD needed. `--px-per-in` for an SVG with no real units (default 96;
Illustrator writes 72).

    $ ncad-laser svg2ljob t.svg -o t.ljob --pens examples/test.lpt
    t.ljob: 2 paths on 2 pens from 2 shapes (50.8 x 25.4 mm)
    origin 0,0; raw: plan it next

**`pdf2ljob IN.pdf|IN.ai -o OUT.ljob [--page N] [--pens FILE.lpt] [--tol MM]`**
Needs qpdf. `pdfstream2ljob STREAM.txt -o OUT.ljob [--mediabox X0,Y0,X1,Y1]
[--pens ...] [--tol ...]` imports a raw, already decompressed content stream
(no qpdf; default box 0,0,612,792).

**`plan IN.ljob -o OUT.ljob [--join-tol MM] [--head X,Y] [--no-2opt] [--order best|lightburn]`**
Joins touching paths, puts inner shapes before outer, orders by pen `order=`,
and minimises travel. `--join-tol` is the join tolerance. `--head X,Y` is where
the head starts, in job mm (default: the job's origin); LightBurn plans from the
beam, which is the origin shifted by the red-dot offset, so to match it give
`--head` as origin plus offset. `--no-2opt` keeps the nearest-neighbour order
without the 2-opt improvement. `--order lightburn` is LightBurn's plain
nearest-neighbour order (one pool, no avoidance of cut-free parts, no 2-opt),
for comparing against a LightBurn file; `best` is the default. In LASER the
same choice is the system variable `LASERORDER` (`best` or `lightburn`), read
when a job is built.

    $ ncad-laser plan examples/card_test.ljob -o planned.ljob
    5 paths in, 2 out (3 joins, 1 closed by joining, 0 skipped)
    cut 151.413 mm, travel 32.425 mm
    1 regions; 0.000 mm of travel still crosses cut-free parts
    estimated 3.9 s (cut 3.6 s, travel 0.3 s)

**`encode PLANNED.ljob -o OUT.rd [--origin WHERE] [--machine-origin bl|br|tl|tr] [--start current|absolute] [--bed W,H] [--pointer-offset X,Y] [--magic N]`**
Writes a swizzled `.rd`. `--origin` overrides the job's origin (x,y in job mm,
or a corner of its box). `--machine-origin` is the machine's origin corner
(default `bl`). `--start current|absolute` overrides the job's start mode.
`--pointer-offset X,Y` is the red-dot offset in mm; encode writes the beam's
file, as LightBurn does (see 5.3). `--bed W,H` gives the bed size so an
absolute-start job that leaves the bed is refused (without it only the home
corner is checked). The power limits apply (the hard 80 default always).

    $ ncad-laser encode planned.ljob -o card.rd --machine-origin tr --pointer-offset 5.2,31.2
    card.rd: 1371 bytes, 1 layers, cut 151.413 mm, travel 36.919 mm, origin 0.000,0.000 (bottom-left), start current

**`rd2txt FILE.rd [options]`** Decodes any `.rd`. `--magic N` (default: guess),
`--machine-origin bl|br|tl|tr`, `--plain` (input already unswizzled),
`--structure` (no offsets or hex: for diffing two files), `--no-offsets`,
`--no-hex`, `--no-positions`, `--no-summary`.

    $ ncad-laser rd2txt card.rd | head -4
    ; card.rd, 1371 bytes, magic 0x88 (guessed)
    000000  D8 12                             Ref Point 0 (current position)
    000002  F0                                Ref Point Set
    000003  F1 02 00                          Enable Block Cutting 0

**`lbrn2ljob IN.lbrn2 -o OUT.ljob [--machine-origin bl|br|tl|tr] [--bed W,H] [--tol MM] [--start current|absolute] [--job-origin bl|br|tl|tr|c]`**
Reads a LightBurn project. LightBurn stores machine coordinates and does not
store Start From, job origin, bed or origin corner in the file, so you give
them. `--job-origin` is LightBurn's job-origin dot (corner or centre).

    $ ncad-laser lbrn2ljob f01.lbrn2 -o f01.ljob --machine-origin tr --job-origin tl
    f01.ljob: 1 layers, 1 paths, origin -557.274,-261.700, start current (LightBurn 1.4.03)

**`ljob2lbrn IN.ljob -o OUT.lbrn2 [--machine-origin ...] [--bed W,H]`**
Writes a project. Refuses air-off and skip layers (no known spelling).

    $ ncad-laser ljob2lbrn f01.ljob -o f01b.lbrn2 --machine-origin tr
    f01b.lbrn2: 1 layers, 1 paths

**`rasterplan IN.png [--size W,H] [--dpi N] [--dither threshold|floyd|jarvis|stucki|bayer|grey] [--speed MM_S] [--accel MM_S2] [--margin MM] [--bidir on|off] [--power MIN,MAX] [--black N] [--white N] [--gamma G] [--invert] [--no-serpentine] [--png OUT.png]`**
Plans an engraving and reports it; `--png` writes the dithered preview. **It
only plans: raster jobs are not encoded to `.rd` or sent yet.**

    $ ncad-laser rasterplan wedge.png --size 50,10 --dpi 254 --dither jarvis
    image 64x16 px -> 500x100 px at 254 dpi, 50.000 x 10.000 mm, dither jarvis, bidirectional
    100 lines of 100 (0 blank skipped), 25095 fired pixels
    overscan 16.000 mm each end (speed 300 mm/s, accel 3000 mm/s2)
    estimated 36.2 s

### Device commands

**`devices [--config PATH]`** Lists lasers.conf.

**`counters WHERE [--csv]`** Read-only. Columns for `--csv`: time, host, card id,
firmware, power-on, work time, laser-on, jobs, travel x, travel y (raw values;
units still to confirm against the panel). Safe first contact.

    $ ncad-laser counters --device blackcat
    card id          0x65106510
    firmware         FAKE-8.01.50
    power-on time    0
    ...
    $ ncad-laser counters --device blackcat --csv >> counters.csv

**`status WHERE`** State (raw), head position, bed size, file progress.

    $ ncad-laser status --device blackcat
    state     22 (0x16, raw)
    position  0.000, 0.000 mm
    bed       900.000 x 600.000 mm
    file      0 of 0

**`watch WHERE [--interval MS] [--duration S] [--csv]`** Polls state, head
x/y and file progress, one line per poll (`--interval` default 500 ms), until
`--duration` or Ctrl-C. A poll with no reply prints `no reply` and the watch
goes on.

       0.000 s      0.5 ms  state 22  head 0.000, 0.000 mm  file 0/0

**`watch WHERE --listen [--duration S]`** Sends nothing after first contact
(UDP: one ENQ; serial: nothing) and logs every datagram or burst of bytes,
timestamped and decoded where possible. It answers whether the controller
pushes status by itself. Start it, then run a job from another program or the
panel; silence is itself the answer.

**`settings WHERE [--from ADDR] [--to ADDR] [--csv] [--timeout MS]`** Read-only
dump of controller memory (only reads are issued). By default every address
MeerK40t's emulator names; `--from`/`--to` (`0x0221`, or wire bytes `04:21`)
read a range. Names are MeerK40t's guesses from reverse engineering, not
confirmed meanings; `*` marks names mentioning red, light, pointer or offset.

    $ ncad-laser settings --device blackcat --from 0x0220 --to 0x0224
      0x0221 [04 21]  Axis Preferred Position 1, Pos X             = 0 (0x0)

**`stop WHERE`** Sends the stop command; does not wait for the handshake.

**`goto X Y WHERE [--machine] [--speed MM_S]`** Moves the head, laser off.
Bed coordinates, or raw machine coordinates with `--machine`. Refuses a target
outside the bed. A rapid move, absolute, laser off; nothing fires.

    $ ncad-laser goto 450 300 --device blackcat
    head -> machine 450.000, 300.000 mm  (bed 450.000, 300.000 mm)

**`jog DIR... | DX DY WHERE [--fine|--coarse|--step MM] [--speed MM_S]`**
Reads the head, adds a step and goes. `DIR` is `+x -x +y -y` (at most one per
axis, in bed directions), step 1 mm, `--fine` 0.1, `--coarse` 10, `--step MM`.
Or explicit `DX DY` in mm with no step flag. `ncad-laser jog +x -y --coarse`
is a diagonal.

**`frame PLANNED.ljob WHERE [--wait]`** Traces the job's box, laser off (no cut
command in the file). Pointer offset not applied.

**`dryrun PLANNED.ljob WHERE [--wait] [--corrupt-sum]`** Uploads the path as a
laser-off job and the controller traces it at once. Pointer offset applied.

`--wait` (frame, dryrun): after the upload, poll the head until it has been
still for 1 s (at most `--duration S`, default 120) and report how long that
took; without it the command returns as soon as the upload is done.

`--corrupt-sum` (dryrun only; refused elsewhere) is a **diagnostic**: it flips
the file's checksum so you can learn whether the controller rejects a bad sum.
It is for hardware probing, not for normal use, and `send` has no such option.

    $ ncad-laser dryrun abs.ljob --device blackcat
    dry run: 1373 bytes in 17 packets (0 NAKs)

**`raw HEX WHERE`** A **diagnostic**: sends one read-only command and prints
the reply. It accepts a memory read written in hex (`raw "DA 00 04 21"`) or the
ENQ (`CE`, which only repeats the connect handshake); anything else is
refused, so it cannot move the head or fire the laser. For poking at what the
controller says about an address.

**`send PLANNED.ljob WHERE [--max-power PCT] [--yes]`** Shows the job and asks
`yes` unless `--yes`.

    job      card-test
    layer 5   speed 100 mm/s, power 10-10%, passes 1, air on
    bounds   475.200,351.200 .. 505.200,381.200 mm
    origin   400.000,250.000 (other), absolute: design coordinates are bed coordinates
    cut      151.4 mm, travel 633.0 mm
    sent: 1373 bytes in 17 packets (0 NAKs)

**`pulse WHERE --power PCT [--ms N] [--method job|key] [--yes]`** **Fires the
laser.** See section 8.

`--speed MM_S` on `goto`/`jog` sends `C9 02` first, as MeerK40t does (600 mm/s
by default). **Unverified on this controller:** that `--speed` changes the
rapid speed, that move option `00` measures from machine zero rather than a
user origin set on the panel, and what `goto --machine 0 0` does (MeerK40t's
emulator treats it as "home").

## 7. Imports

Three commands add geometry to the drawing. Each makes layers and entities
and, where it can, a pen table for LASER.

**`SVGIN`** prompts `SVG file to import`. Units come from the SVG's
width/height and viewBox; with no real units a pixel is 1/96 in (system
variable `SVGPXIN`; Illustrator writes 72). Y is flipped so the SVG's
lower-left is the origin. Output is polylines and splines (curves stay
curves). Fills are outlined with a warning. A stroke's colour (else fill)
becomes the nearest ACI, which is the pen; a named group becomes a layer.

    SVGIN: 1 polylines, 1 splines, 0 new layers (50.8 x 25.4 mm)

**`PDFIN`** prompts `PDF or AI file to import`, then `Page <1>`. Needs qpdf at
build time; otherwise:

    ; PDFIN: built without qpdf; PDF and AI import need NLASER_WITH_QPDF

PDF units are 1/72 in, converted to mm. Paths only: text, shadings, images and
form XObjects are skipped with a warning each. Illustrator layers
(optional-content groups) become layers. Pre-CS Illustrator files are
PostScript, not PDF, and are refused. Page rotation and CropBox are not
applied.

**`LBRN2IN`** prompts `LightBurn project to import`. Each CutSetting becomes
a layer named for LightBurn's layer and coloured with the nearest unused
ACI; its cut settings become the LASER pen table (so pen = ACI matches the
entities). One undo step. `LBRN2CORNER` (default `tr`, or the chosen
device's `origin_corner`) says which corner the project's machine counts
from. Only Path and Ellipse shapes on Cut layers are read; anything else is
refused with a line number. LightBurn's layer colours come from memory of its
palette beyond layer 2 (red) and are unverified; a wrong guess only changes
the ACI colour, not the cut settings.

    LBRN2IN: 1 polylines, 0 circles, 0 ellipses, 1 new layers, 1 pens for LASER

Colour to pen, in all of these: the pen is the entity's ACI colour. Set
colours so each cutting operation has its own pen, then give each pen speed
and power in the table.

Other system variables: `LASERDEV` (device name), `LASERPENS` (pen table
path), `LASERTOL` (curve tolerance, mm), `LASERUNITS` (`mm` or `in`: what a drawing unit is), `LASERCONFIRM` (1 = ask before Send
and Pulse), `SVGPXIN`, `LBRN2CORNER`. They are installation settings, not
drawing content: `NEW` and `OPEN` do not reset them and none is written to the
DXF.

## 8. Safety

**What can fire the laser, and only these:**

| Operation | Fires? |
| --- | --- |
| `send` / LASER Send | Yes: runs the job at the pens' power. Asks `yes` (CLI) or `Yes/No <No>` (LASER). |
| `pulse` / LASER Pulse | Yes: asks first. |
| `frame`, `dryrun`, `goto`, `jog`, Move | No. Frame and dryrun files contain no cut command and every power is 0; goto/jog are a rapid move with light off. |
| `status`, `counters`, `watch`, `settings`, `devices`, `rd2txt` | No (reads only). |
| `stop` | No: it stops motion. |
| `plan`, `encode`, Save | No: files only. |

**Confirmations.** `send` shows the job (layers, bounds, origin, length) and
asks; Enter is No. `pulse` asks. `--yes` skips the question for `send` and
`pulse`, so do not put it in a habit or an alias. In LASER, `LASERCONFIRM`
defaults to 1 (ask).

**Limits.** Every layer power above `max_power` or `power_limit` (the smaller)
is refused, before anything is uploaded. A job outside the bed is refused
(current mode: with the head's position; absolute: against the bed). The job
size is checked against the configured bed before Send is offered. Power 0 is
what an unlisted pen gets.

**Pulse.** `--method job` (default) sends a one-cut micro-job at the head's
position at `--power` and 1 mm/s, so the head moves exactly `ms/1000` mm
along design +X (printed before it asks). `--ms` is 10..2000. The power is
refused above the cap:

    pulse    FIRES THE LASER on 127.0.0.1, method job
    power    5% (operator max 60%, hard limit 80%)
    hold     one cut at 1 mm/s for 300 ms: the head moves exactly 0.300 mm along design +X from where it is now (origin_corner/flip_x may mirror this on the machine)

`--method key` presses the panel's Pulse key over UDP; the power is whatever
the panel is set to and cannot be checked, so it is refused unless
`max_power` and `power_limit` are both 100. It is not available over serial,
and no real controller has been tried with it.

**Stop.** `ncad-laser stop WHERE` and LASER `STOp` send the controller's stop
command and do not wait for the handshake (if the handshake fails it tries
anyway). Over serial the stop is not acknowledged. Neither replaces the lid
switch, the panel's Stop and the e-stop; keep a hand near them on a first run.

**What the tool cannot know.** The controller is not asked before a job
starts whether the lid is shut, the air is on, or the material is where you
think. `Frame` first, `Dryrun` second, then Send, on anything new.

### Probing a real controller: `tools/laser_edge_cases.sh`

    tools/laser_edge_cases.sh --host 192.168.1.100
    tools/laser_edge_cases.sh --serial /dev/cu.usbserial-XXXX

An interactive script that runs the protocol edge cases ten steps at a time
(Enter runs, `s` skips, `q` quits) and logs everything with timestamps to a
`laser-edge-DATE/` directory: a baseline of counters, status, settings and a
`raw` read; listening while idle; reads and a second job during a dry run;
`stop` mid-job; a dry run with a bad sum (`--corrupt-sum`); a UDP packet-size
sweep (`--chunk` 200 to 1400); a position check with `goto` and `jog`; an
unplugged cable; and an open lid. **The laser cannot fire**: it sends only
reads, laser-off dry runs, goto/jog and stop, and its temporary lasers.conf
sets `max_power 0`. The head does move (a 40 mm square from the bed centre at
20 mm/s), so clear the bed and keep a hand near the E-stop. At the end it
prints a checklist of what to report back.

## 9. Troubleshooting

**No `/dev/cu.usbserial-*` device.** Check, in order: a data cable (many cheap
USB-B cables are charging-only); the side USB-B port, not the top USB-A stick
port; the controller is powered; `ls /dev/cu.*`. Nlaser says exactly what it
tried:

    ncad-laser: cannot open /dev/cu.usbserial-A904TTIM: No such file or directory

The device name changes if you move the cable to another port; update
`serial =`.

**Charging-only cable.** The symptom is no device and no change in
`ls /dev/cu.*` when you plug in. Swap the cable.

**`serial` commands send but nothing seems to happen.** Over serial nothing is
acknowledged and reads may go unanswered. Use `status` and `watch` to see
whether the controller answers at all; if reads work, placement checks work.

**LightBurn holds the UDP port.** Over Ethernet only one program per computer
can use the controller's reply port (40200). If a command times out or the
port is in use, quit LightBurn completely and retry.

**The panel shows the manual speed and power, not the job's.** Known and expected:
the panel's speed and power fields are its manual settings, while a job
carries its own settings per layer in the file. Check what the job
carries with `rd2txt` on the file (the `Speed Laser`, `Min/Max Power` lines) or `Send`'s
layer list. This is expected and not a fault.

**The job "would leave the bed".** In Current mode the check uses the head's
current position: move the head with `jog`/`goto`, or use Absolute. Remember
the box includes the pointer offset. If a bed size cannot be read, set `bed`
in lasers.conf.

**A refusal over power.** Read which limit stopped it. `max_power` is
yours to change (lasers.conf or `--max-power`). `power_limit` is not available
on the command line, by design; edit lasers.conf if the machine's owner agrees.

**Qt toolbar cursor crash in `ncad_gui`.** A known crash on the toolbar
cursor in the GUI; it is a NotoCAD GUI bug, not an Nlaser one. It does not
touch the laser: nothing is sent unless a command sent it, and Stop and the
panel work independently. After it, restart the GUI, and if a job was running,
treat it as running (watch it, or Stop).

**Parse errors in lasers.conf** name the file and line (see
`tests/fixtures/bad_lasers.conf` for what they look like).

## 10. Status: what has been tried on hardware

The repository's own notes, not a promise. "Verified" means checked against
real LightBurn output or a real machine, as stated.

| Feature | Status |
| --- | --- |
| `.rd` encoder against LightBurn's files | Verified byte for byte for the samples in `tests/fixtures/lightburn/` (current-position square, 2 and 3 passes, centre origin, bottom-right origin, absolute start). LightBurn's absolute-start file "runs correctly on the machine" (hardware, by LightBurn). Circles (f07) match in structure but not byte for byte. |
| Red-dot offset K = (5.2, 31.2) | Recovered from LightBurn's files and confirmed with the user as the device setting. "Frame is not offset" is the user's description, not captured. |
| Decoder (`rd2txt`) | Tested on LightBurn and MeerK40t files. |
| UDP transport (ACK/NAK, resend, chunking) | Tested against an in-memory controller and loopback. Not yet against a real controller. |
| USB serial transport | Tested against an in-memory controller and a pseudo-terminal. Not yet against a real controller. No acknowledgements by design of the framing. |
| `goto`, `jog`, Move | Tested against fakes. That `--speed` changes the rapid speed, and the meaning of option `00`, are unverified. |
| `frame`, `dryrun`, `send`, `pulse` | Tested against fakes. Not yet recorded as run on a machine. |
| `pulse --method key` | Packets follow MeerK40t's emulator; no real controller tried. |
| Absolute placement (bed-coordinate mapping, bed size) | Not yet run on a machine. |
| `settings`, `counters`, `status`, `watch` | Tested against fakes; register names are MeerK40t's guesses; counter units unconfirmed. |
| `.lbrn2` reader | Six LightBurn samples round-trip to the byte; shapes other than Path and Ellipse and layers other than Cut are refused. |
| `.lbrn2` writer | Checked by round trip and against the samples' element order; **never opened in LightBurn**. |
| LBRN2IN layer colours | Index 2 = red is evidenced; the rest is from memory. |
| SVG import | Tested with fixtures. |
| PDF / AI import | Tested on content streams; the qpdf container path is built only where qpdf is found. |
| Raster | `rasterplan` plans and previews only; no encoder, no send. |
| Rotated `LASER` UCS | Not applied (origin only). |
| Networking beyond one controller / remote access | Not implemented. |

Update this table as things get run on the machine; that is its purpose.
