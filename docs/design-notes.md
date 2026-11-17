# Design notes

Decisions that are easy to forget and cheap to revisit. Newest first.

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
