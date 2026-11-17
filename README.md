# Nlaser

Drive a Ruida controller laser (Black Cat, RDC6442S) from NotoCAD, and from the
command line. Its `.rd` output is byte-compatible with LightBurn's; LightBurn
`.lbrn2` projects can be read and written.

**Start with [docs/user-guide.md](docs/user-guide.md)**: setup, lasers.conf,
the LASER command, placement, every `ncad-laser` subcommand, safety,
troubleshooting and what has been verified on hardware.

The design is in `NotoCADLaserBackend—Spec.md`. This tree is the headless core
(`ncad_laser_core`), the file-driven tool `ncad-laser`, and the `LASER`,
`SVGIN`, `PDFIN` and `LBRN2IN` commands that NotoCAD (branch `laser`) loads.

## Build

    cmake -S . -B build -G Ninja
    cmake --build build
    ./build/tests/nlaser_tests

With NotoCAD (adds `dxf2ljob`; its tree beside this one, or say where):

    cmake -S . -B build -G Ninja -DNLASER_WITH_NOTOCAD=ON [-DNLASER_NOTOCAD_DIR=../NotoCAD]
    cmake --build build
    ./build/tests/nlaser_ncad_tests

Needs CMake 3.25. PDF/AI import uses qpdf when found
(`-DNLASER_WITH_QPDF=AUTO|ON|OFF`). NotoCAD's own build picks this tree up
(`NCAD_LASER_DIR`, default `../Nlaser`) to provide the `LASER` command.

## Quick tour

    ncad-laser plan   examples/card_test.ljob -o planned.ljob
    ncad-laser encode planned.ljob -o card.rd --machine-origin tr --pointer-offset 5.2,31.2
    ncad-laser rd2txt card.rd                       # decode any .rd, LightBurn's too
    ncad-laser counters --device blackcat           # read-only first contact
    ncad-laser frame    planned.ljob --device blackcat   # laser off
    ncad-laser send     planned.ljob --device blackcat   # asks for 'yes'

No laser handy: `build/tests/nlaser_fakectl --port 50299` is a fake controller
on a loopback port; use `--host 127.0.0.1 --port 50299 --local-port 0`.

Devices are named in `~/.config/ncad/lasers.conf` (or
`$XDG_CONFIG_HOME/ncad/lasers.conf`); the keys are in the guide. Power is
capped twice: `max_power` (operator) and `power_limit` (hard, default 80,
file-only).

## Documents

- `docs/user-guide.md`: the user guide.
- `docs/design-notes.md`: decisions that are easy to forget.
- `docs/lightburn-fixtures.md`: what LightBurn's files show (frames, offsets, bed).
- `docs/lbrn2-format.md`: the `.lbrn2` subset understood.

Portions are ported from MeerK40t (MIT); see `third_party/meerk40t/NOTICE`.
