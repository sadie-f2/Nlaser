# Design notes

Decisions that are easy to forget and cheap to revisit. Newest first.

## 2026-11-19: Laser mode in the GUI: run the machine, not the design (Sadie)

`LASER` is already a mode (the modal command, sub-modes Pens, Move, Order;
user guide §4). In the GUI it becomes a mode of the window too. Entering it
steps out of design and into running the laser hardware and the cutting
process. A menu or toolbar shows ONLY in laser mode. Every control issues the
`LASER` keyword it stands for, so the command line shows what happened (the
spec's rule).

- **Job control:** Start (Send), Stop, Pause and Resume as buttons. Stop is
  always enabled while a job runs.
- **Jog X, Y and Z, with a velocity slider.** Z is new: the command's Move
  has only ±X/±Y today. The slider sets the jog speed; Fine, Coarse and Step
  remain for stepped moves.
- **Goto by pick:** a button, then pick a point in the workspace, and the
  head moves there (Move Goto's two meanings, 11-16, apply unchanged).
- **A button for every other LASER keyword** (Frame, Dry run, Status,
  Watch, Counters, Origin, Preview, Save...).
- **Pens is its own mode within laser mode**: the pen-table grid of the
  spec's GUI section, entered from laser mode, with its own controls.
- **Later, maybe:** dynamic control of power on a running job. This needs a
  live-override path to the controller, so check what Ruida allows (and how
  it interacts with `power_limit`, which no live control may exceed) before
  designing it.

**Not a separate viewport: the same window's viewport, in laser mode**
(Sadie; this replaces the spec's "separate view in the Qt shell"). In laser
mode the viewport paints the machine: the bed, the job, real-time status and
the head's position. Drawing edits are probably not allowed there. One way to
think of it, not yet settled: a **toggle** between "display the drawing" and
"display the laser cutter's setup and operation", so the person can look back
at the drawing without leaving laser mode, and the drawing cannot be changed
under a running job by accident.

**Where it is built:** in NotoCAD-solids, when the hold lifts (the original
NotoCAD gets no backport). Nothing here needs the solids library; laser
cutting is 2D.

**Raster may go ahead now,** designed, built and tested against the NotoCAD
0.2.x repository (branch `laser`) and this tree. It touches nothing in
-solids, so the hold does not apply to it. The spec's plan stands: learn the
encoding from LightBurn's `.rd` files (a greyscale wedge, a dithered photo,
a 1-pixel line pattern), then confirm at low power on the machine.

## 2026-11-18: Contents and placement are orthogonal; selection never overrides placement

Decided by the user. What a job CONTAINS (the Area: Extents, Window, Display,
Limits, Selection) and WHERE it is placed (Start from Absolute, User or
Current; the origin: UCS LASER or a picked point, or a point of the job's box)
are independent settings, and every placement works with every area.

- Choosing Selection changes the contents only. It does not change the start
  mode, the origin or the box anchor, and it never turns an anchor on. In
  Absolute a selection is cut at its own drawing coordinates on the bed.
- The nine-point box anchor (LightBurn's job origin dots) is one more origin
  choice beside UCS LASER, chosen with ORIgin Box or LASERANCHOR. It is
  resolved against the bounds of whatever is being cut, but only when chosen.
  Not applied in Absolute (the drawing is the bed); it says so.
- An existing selection is offered as the default ANSWER of the Area prompt,
  shown in it, never applied silently.
- Pinned by tests: for each start mode, Extents and Selection of the same
  drawing differ in contents only.
- The shop's habit (cut the selection from the user origin with a box anchor)
  is a local convenience built from these independent pieces, not the design.
- Engine limit: CommandEngine::begin() demotes a selection made before a
  command to Previous, so a pickfirst set is reached with Selection, then
  Previous, not as an implied default at the first Job. An implied default
  works once a selection exists inside the LASER mode.

## 2026-11-17: The operator stays with the machine; Ethernet point to point only

A2's safety rule: the member running a job stays with the machine -- laser
cutters catch fire. Lasers are never put on the facility network. Ethernet
is the preferred link, but as a direct cable from the operator's laptop to
the controller (static addresses, nothing else on the wire); USB also works.
So the UDP transport is the main path, while nothing is designed for remote
use: no sending from another room, no job queues, no "notify me when done"
in place of being there. Features that help the person standing at the
machine (Pause, Stop, Watch, Frame) come first.

## 2026-11-16: The hard power limit is file-only

`power_limit` (percent, default 80, per device in lasers.conf) is a hard
ceiling kept apart from `max_power`. `max_power` is the operator's cap for a
material or a day; `power_limit` protects the tube while it is being
maintained, and nobody should be able to wave it away mid-job.

- **Built into the code, not the config.** The default is 80 in `DeviceConfig`
  and in `RdJob`, so it holds with no lasers.conf and for `ncad-laser encode`,
  which writes an .rd for a USB stick with no device at all.
- **Enforced in `encode_rd`.** Every layer power (min, max, tube 1 and tube 2)
  above the limit is refused, so send, dryrun, encode and LASER Send/Save .rd
  cannot bypass it by construction. Dry runs and frames write power 0 and are
  not held to it. The effective operator cap is min(`max_power`,
  `power_limit`); refusals say which one stopped the job.
- **Deliberately no flag and no prompt.** A command-line flag or a LASER
  prompt is one keystroke from being raised by habit, a script or an
  autocompleted history line. Editing lasers.conf is a conscious act on the
  machine's owner's side, which is the point of a maintenance setting. The
  price is that raising it means leaving the tool; that is intended.
- **Key pulse.** The panel's Pulse key fires at a power set on the controller
  that cannot be read or capped, so it is refused unless both limits are 100.

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
