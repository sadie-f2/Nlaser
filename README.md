# Nlaser
Drive a Ruida controller laser from NotoCAD interface.

The design is in `NotoCADLaserBackend—Spec.md`. This tree is the headless core
(`ncad_laser_core`) and the file-driven tool, `ncad-laser`.

## Build

    cmake -S . -B build -G Ninja
    cmake --build build
    ./build/tests/nlaser_tests

With NotoCAD (for `dxf2ljob`; its source tree beside this one, or say where):

    cmake -S . -B build -G Ninja -DNLASER_WITH_NOTOCAD=ON [-DNLASER_NOTOCAD_DIR=../NotoCAD]
    cmake --build build
    ./build/tests/nlaser_ncad_tests

NotoCAD's `ncad_core` is built from its own CMakeLists as a subproject, into
its own target; `ncad_laser_core` never links it. Needs CMake 3.25.

## Use

    ncad-laser dxf2ljob drawing.dxf -o raw.ljob [--pens card.lpt] [--tol 0.01] [--origin x,y]
    ncad-laser plan   examples/card_test.ljob -o planned.ljob
    ncad-laser encode planned.ljob -o card_test.rd [--origin bl|br|tl|tr|x,y] [--start current|absolute]
    ncad-laser rd2txt card_test.rd             # decode any .rd, LightBurn's too
    ncad-laser rd2txt a.rd --structure > a.txt  # for diffing two files

Talking to a controller (UDP; the controller listens on 50200 and answers on
40200, so only one program per computer can use it at a time):

    ncad-laser counters --host 192.168.1.100           # read-only; safe first contact
    ncad-laser counters --host 192.168.1.100 --csv >> counters.csv
    ncad-laser status   --host 192.168.1.100
    ncad-laser frame    planned.ljob --host 192.168.1.100   # laser off: no cut command in the file
    ncad-laser dryrun   planned.ljob --host 192.168.1.100   # the whole path, laser off (every power 0)
    ncad-laser watch    --host 192.168.1.100 --interval 250 --duration 60 [--csv]
    ncad-laser pulse    --host 192.168.1.100 --power 5 --ms 300     # FIRES THE LASER; asks for 'yes'
    ncad-laser send     planned.ljob --host 192.168.1.100   # shows the job, asks for 'yes'
    ncad-laser stop     --host 192.168.1.100
    ncad-laser goto 450 300 --serial /dev/cu.usbserial-1410          # bed coordinates
    ncad-laser goto 12.5 7 --machine --serial /dev/cu.usbserial-1410  # raw machine coordinates
    ncad-laser jog +x --serial ...     # 1 mm to the right; --fine 0.1, --coarse 10, --step MM
    ncad-laser jog +x -y --coarse --serial ...   # a diagonal
    ncad-laser jog -3.5 0.25 --serial ...        # explicit mm (DX DY), no step flag

`goto` and `jog` move the head with the laser off (a rapid move, `D9 10`,
options byte `00`: absolute, from the origin, no light, nothing fires). `goto
X Y` takes BED coordinates (0,0 at the bed's bottom-left, X right, Y up; a
machine that counts from its right or top edge needs the bed size, from
lasers.conf `bed`, `--bed W,H` or the controller), or raw machine coordinates
with `--machine`. `jog` reads the head position, adds the step in bed
directions (`+x` is always rightwards on the bed, whatever the machine's
origin) and goes there. Both refuse a target outside the bed. `--speed MM_S`
sends `C9 02` first, the speed command MeerK40t sends before its own rapid moves
(600 mm/s by default); `C9 03` (Speed Axis, 15 mm/s in LightBurn's headers) and
`C9 06` (Axis Move Speed) are known but MeerK40t never uses them for a jog.
**Unverified on this controller: that `--speed` changes the rapid speed, that
option `00` measures from machine zero rather than a user origin set on the
panel, and what `goto --machine 0 0` does (MeerK40t's emulator treats it as
"home").** Over serial the move is not acknowledged.

CSV columns: time, host, card id, firmware, power-on, work time, laser-on,
jobs, travel x, travel y (raw values; units still to confirm on the panel).

`watch` prints one line per poll (time, reply latency, raw state, head x/y,
file total/progress) until `--duration` or Ctrl-C; a poll that gets no reply
is printed as `no reply` and the watch goes on, so it can show whether the
controller answers reads while a job runs, and how fast. `dryrun` is the job
`send` would upload, encoded travel-only: every cut a move, every power 0, no
cut command in the file (the LASER command has the same `Dryrun` keyword).

**`pulse` fires the laser.** Both methods ask for `yes` (or `--yes`), and
`--ms` is 10..2000.

- `--method job` (default): a one-cut micro-job at the head's position, at
  `--power PCT` and 1 mm/s, so the head moves exactly `ms/1000` mm along design
  +X (printed before it asks). The power is the one you give, refused above the
  device's `max_power`. Works over UDP and serial.
- `--method key`: the panel's Pulse key, pressed and released over UDP on the
  keypad port (`keypad_port`, default 50207). The power is whatever is set on
  the controller's panel; `--power` is ignored and the tool cannot check it
  against `max_power`, so it refuses when a device's `max_power` is below 100.
  Not available over serial. The packets follow MeerK40t's key codes and its
  emulator (plain, no swizzle or checksum); no real controller has been tried.

No laser handy: `build/tests/nlaser_fakectl --port 50299` is the test suite's
fake controller on a loopback port; point ncad-laser at it with
`--host 127.0.0.1 --port 50299 --local-port 0`.

Devices can be named instead of given by address. `lasers.conf` lives at
`$XDG_CONFIG_HOME/ncad/lasers.conf` (else `~/.config/ncad/lasers.conf`; `--config
PATH` overrides). One `[name]` section per device, `#` or `;` for comments:

    [bench]
    driver = ruida          # the only driver so far
    host = 192.168.1.100    # or, over USB: serial = /dev/cu.usbserial-XXXX (baud, hw_flow optional)
    magic = 0x88            # `auto` is not implemented yet
    port = 50200
    local_port = 40200
    keypad_port = 50207     # pulse --method key
    max_packet = 1000
    max_power = 60          # send refuses any layer above this
    ack_timeout_ms = 2000
    bed = 600 400           # W H in mm; stored, not used yet
    origin_corner = bl      # bl br tl tr: the machine's origin; applied by send, frame
    flip_x = false          # extra axis mirrors, applied the same way
    flip_y = false

    ncad-laser devices                       # list what is configured
    ncad-laser counters --device bench
    ncad-laser send planned.ljob --device bench --max-power 30   # flags override the file
    ncad-laser status                        # no --host/--device: fine if exactly one device

Unknown keys and malformed values are errors that name the file and line.

Or over the controller's USB port, which appears as a serial device
(`/dev/cu.usbserial-*` on macOS, `/dev/ttyUSB0` on Linux); give `--serial DEV`
in place of `--host` (add `--baud N` or `--no-flow` if needed):

    ncad-laser counters --serial /dev/cu.usbserial-1410
    ncad-laser send     planned.ljob --serial /dev/ttyUSB0

Serial has no checksum and no acknowledgements (MeerK40t's reverse-engineered
framing; see the comment block in `ruida_link.hpp`), so a send is not
confirmed by the controller. Untested on hardware.

`dxf2ljob` writes a raw job: one path per polyline NotoCAD draws, the pen
being its ACI colour. Each pen gets a `layer` line from the `.lpt` pen table
(`layer` lines in `.ljob` syntax); a pen the table does not list gets power
0, with a warning. The origin is the drawing's `LASER` UCS, else 0,0.
Drawing units are taken as mm. Not yet: wide polylines and SOLIDs as raster
(they cut as centre lines and outlines, with a warning), a rotated `LASER` UCS.

`.rd` output starts from the head's current position (LightBurn's "Current
Position"): the job's `origin` goes where the head is. Before `send`, `frame`
and `dryrun` upload such a job, the head position and the bed size (lasers.conf
`bed W H`, else memory 00 26 / 00 36) are read and the job's box is placed at
the head: if it would leave the bed the command is refused, with the numbers.
If the head or the bed cannot be read (serial need not answer) it WARNS and
goes on; the warning is printed and is never silent.

`--start absolute` (or `start=absolute` in the `.ljob`): the design
coordinates ARE bed coordinates, (0,0) at the bed's bottom-left, X right, Y
up, mm. They are mapped to machine coordinates with the bed size (a right-hand
origin gives x_m = W - x, a top origin y_m = H - y, then flips). A machine
whose origin is not bottom-left needs the bed size (`encode --bed W,H`,
lasers.conf `bed`, `--bed`, or the controller's report) or the job is
refused; any coordinate outside the bed is refused ("leaves the bed").
`docs/lightburn-fixtures.md` says how this relates to LightBurn's workspace.

## Status

- `.rd` encoder reproduces LightBurn's own files byte for byte
  (tests/fixtures/lightburn_*.rd); not yet run on a machine.
- UDP transport (ACK/NAK, resend on NAK only, chunking at command
  boundaries) tested against an in-memory controller and over loopback; not
  yet against a real controller.
- USB serial transport (framing from MeerK40t's work-in-progress USB
  support) tested against an in-memory controller and a pseudo-terminal; not
  yet against a real controller.
- Start modes: current position (default) and absolute coordinates (f03
  reproduced byte for byte; `.ljob` `start=`, `encode --start`, LASER Origin);
  absolute placement (bed-coordinate mapping, see above) not yet run on a
  machine. Not yet: raster.

Portions are ported from MeerK40t (MIT); see `third_party/meerk40t/NOTICE`.
