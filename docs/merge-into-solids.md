# Merging laser into NotoCAD-solids: anticipated challenges (notes, HELD)

Written 2026-11-18 by the steward, from a read of the repos, not a trial. Sadie is holding the merge, and UI integration waits until after it.

**What moves:** NotoCAD `laser` is 3 commits on `main`, about 720 lines over 30 files.
- `06ac77a`: extension hooks (registered commands and system variables).
- `3b52422`: the renderer's `begin_entity` names its source entity, and polyline width reaches the renderers.
- `b1a5388`: `NCAD_WITH_LASER` links LASER into `ncad` and `ncad_gui`.

Everything else is in Nlaser.

**Challenges:**
1. **Drift.** solids has 761 commits since the merge base (dc16892, 2026-08-17). The P1 partition moved `src/core/*` to `src/draft/*` (dash, plot, pick, polyline, commands, sysvar…). Rename detection should carry most hunks; expect hand-merges in `commands.cpp`, `sysvar.cpp` and the CMake files.
2. **Two extension seams.** solids already registers its commands through `include/ncad/drawing_extension.hpp` (used by `solids_install.cpp`). Laser's `06ac77a` likely duplicates it. Port LASER to the solids seam; don't rebase that commit.
3. **The renderer contract.** solids' renderers have grown (tint, edge band, GL probes, offscreen ctest probes). `3b52422`'s `begin_entity` source and polyline-width change must fit every solids renderer and keep the render_diff and GL probes green.
4. **Sysvars.** solids has sticky system variables in one readable file (brief 81). Laser's sysvars must be registered there, under the existing persistence rules.
5. **The floor discipline.** The merge must pass the solids floor: both builds 0/0, offscreen `ctest -LE suite`, 0 warnings, and `-ffp-contract=off` tree-wide (DET-NCAD). The laser tests get ported into the sharded families.
6. **The Qt segfault on Mac** seen in the old line. It's probably fixed in solids, but that's not confirmed: no matching commit was found by message. Reproduce on solids after the merge; the backtrace would let us find or confirm the fix.
7. **Hardware-facing safety.** The power limit (encoder-enforced, default 80), the red-dot offset and the dry run must survive the port unchanged; a test pins each. No send path runs in CI.
8. **Licence and privacy.** Nlaser is BSD-3, like every repo. Device details and any private fixtures stay out of the public history.

**Cheapest de-risk, when wanted:** a Sonnet dry run in a scratch worktree. Cherry-pick the 3 commits onto solids, port the registration to `drawing_extension.hpp`, build `NCAD_WITH_LASER` on the CLI only, and list the conflicts and breakages. Merge nothing.
