# Design notes

Decisions that are easy to forget and cheap to revisit. Newest first.

## 2026-11-16: Move Goto has two meanings, set by Start From

Move's Goto picks a point in the drawing; what that means depends on how the
job is placed. Jog is always a displacement from the head in bed directions.
All Move operations are pointer moves: laser off, no red-dot offset, like
frame and the transport's goto.

- **Start From Absolute:** the drawing is the bed, so the point is a bed
  coordinate (`MoveFrame::Bed`), the same place an absolute job would cut it.
- **Start From Current (default):** the job origin is wherever the head was.
  The head position is read once, on the first Move operation after entering
  Move, and kept as the anchor in machine coordinates. Goto then moves to
  anchor + machine-mapped (point - UCS LASER origin) (`MoveFrame::Machine`;
  the displacement is mapped by the machine's axis signs, so a top-right
  machine goes the other way in machine counts). The anchor is never re-read:
  repeated Gotos do not drift, jogs do not change it, and leaving and
  re-entering Move takes a new one. If the position cannot be read (a serial
  link without replies), Goto is refused with a message; jog still works.

## 2026-11-16: LASER errors are not visible to AutoLISP

The `LASER` command is modal (R12 `LAYER`/`PEDIT` style): a keyword that
fails, such as no reply from the controller or a refused Send, prints
`error: ...` and returns to the `Laser` prompt instead of ending the command.
That is right at the keyboard, but it means `(command "LASER" ...)` always
completes, and a script cannot tell that a step failed.

**Decision:** accepted. Laser cutting is run by a person at the machine,
not driven from AutoLISP (user, 2026-11-16).

**If that changes:** add a registered system variable, e.g. `LASERERR`,
cleared when each keyword starts and set to the error text when it fails,
so a script can test `(getvar "LASERERR")` after each step. Nothing else in
the command needs to change; the registry for system variables already
exists in NotoCAD (branch `laser`).
