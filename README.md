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

`.rd` output starts from the head's current position (LightBurn's "Current
Position"): the job's `origin` goes where the head is.

## Status

- `.rd` encoder reproduces LightBurn's own files byte for byte
  (tests/fixtures/lightburn_*.rd); not yet run on a machine.
- Not yet: multiple passes, absolute start modes, raster, UDP transport.

Portions are ported from MeerK40t (MIT); see `third_party/meerk40t/NOTICE`.
