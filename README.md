# Nlaser
Drive a Ruida controller laser from NotoCAD interface.

The design is in `NotoCADLaserBackend—Spec.md`. This tree is the headless core
(`ncad_laser_core`) and the file-driven tool, `ncad-laser`.

## Build

    cmake -S . -B build -G Ninja
    cmake --build build
    ./build/tests/nlaser_tests

## Use

    ncad-laser plan   examples/card_test.ljob -o planned.ljob
    ncad-laser encode planned.ljob -o card_test.rd [--origin bl|br|tl|tr|x,y]
    ncad-laser rd2txt card_test.rd             # decode any .rd, LightBurn's too
    ncad-laser rd2txt a.rd --structure > a.txt  # for diffing two files

Talking to a controller (UDP; the controller listens on 50200 and answers on
40200, so only one program per computer can use it at a time):

    ncad-laser counters --host 192.168.1.100           # read-only; safe first contact
    ncad-laser counters --host 192.168.1.100 --csv >> counters.csv
    ncad-laser status   --host 192.168.1.100
    ncad-laser frame    planned.ljob --host 192.168.1.100   # laser off: no cut command in the file
    ncad-laser send     planned.ljob --host 192.168.1.100   # shows the job, asks for 'yes'
    ncad-laser stop     --host 192.168.1.100

CSV columns: time, host, card id, firmware, power-on, work time, laser-on,
jobs, travel x, travel y (raw values; units still to confirm on the panel).

No laser handy: `build/tests/nlaser_fakectl --port 50299` is the test suite's
fake controller on a loopback port; point ncad-laser at it with
`--host 127.0.0.1 --port 50299 --local-port 0`.

Devices can be named instead of given by address. `lasers.conf` lives at
`$XDG_CONFIG_HOME/ncad/lasers.conf` (else `~/.config/ncad/lasers.conf`; `--config
PATH` overrides). One `[name]` section per device, `#` or `;` for comments:

    [bench]
    driver = ruida          # the only driver so far
    host = 192.168.1.100
    magic = 0x88            # `auto` is not implemented yet
    port = 50200
    local_port = 40200
    max_packet = 1000
    max_power = 60          # send refuses any layer above this
    ack_timeout_ms = 2000
    bed = 600 400           # W H in mm; stored, not used yet
    origin_corner = bl      # bl br tl tr; stored, not applied yet
    flip_x = false          # stored, not applied yet
    flip_y = false

    ncad-laser devices                       # list what is configured
    ncad-laser counters --device bench
    ncad-laser send planned.ljob --device bench --max-power 30   # flags override the file
    ncad-laser status                        # no --host/--device: fine if exactly one device

Unknown keys and malformed values are errors that name the file and line.

`.rd` output starts from the head's current position (LightBurn's "Current
Position"): the job's `origin` goes where the head is.

## Status

- `.rd` encoder reproduces LightBurn's own files byte for byte
  (tests/fixtures/lightburn_*.rd); not yet run on a machine.
- UDP transport (ACK/NAK, resend on NAK only, chunking at command
  boundaries) tested against an in-memory controller and over loopback; not
  yet against a real controller.
- Not yet: multiple passes, absolute start modes, raster, USB serial,
  machine transforms from lasers.conf (origin corner, flips, bed size are
  parsed and stored but not applied).

Portions are ported from MeerK40t (MIT); see `third_party/meerk40t/NOTICE`.
