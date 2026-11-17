# Design notes

Decisions that are easy to forget and cheap to revisit. Newest first.

## 2026-11-17: The operator stays with the machine; no networked lasers

A2's safety rule: lasers are not put on the network, because the member
running a job must stay with the machine -- laser cutters catch fire. USB is
the link in use. The UDP transport stays (it is tested and other shops use
Ethernet), but nothing is designed around it: no remote send, no queueing
jobs from another room, no "notify me when done" in place of being there.
Features that help the person standing at the machine (Pause, Stop, Watch,
Frame) come first.

## 2026-11-17: Pause/Resume, Watch and Device Offset in LASER

- **Pause/Resume** are `pause()`/`resume()` on `LaserTransport` (D8 02 and
  D8 03, MeerK40t's `PAUSE_PROCESS` / `RESTORE_PROCESS`), sent like Stop with no
  handshake wait. Unverified on our controller; every help text and the user
  guide say so until hardware confirms.
- **Watch** polls on the UI thread. NotoCAD's command model has no way for a
  command to see a keypress while it runs, so LASER Watch is bounded by a
  prompted number of seconds (default 30) and also ends when the head has been
  still and the state idle for about a second. While it runs the whole
  application is blocked. **The `-solids` GUI panel will move it to a worker
  thread** (poll off the UI thread, post lines to the event loop, cancel from a
  button). Time is counted in polls so tests run it with a fake sleep. The state
  words and `n/a` reading are shared with the CLI in `status_words.hpp`.
- **Device Offset** is the only lasers.conf key LASER writes. It edits that one
  line in place (or adds it at the end of the section), keeps every other byte,
  and writes through a temp file and rename. `power_limit` has no such path and
  must not get one: the rule below stands.

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
