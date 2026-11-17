#!/bin/sh
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, Sadie Forbes
#
# Protocol edge cases on a real Ruida, SAFELY, everything logged.
#
#   tools/laser_edge_cases.sh --serial /dev/cu.usbserial-XXXX
#   tools/laser_edge_cases.sh --host 192.168.1.100
#
# The laser cannot fire here: the only things sent are memory reads, laser-off
# dry runs (no cut command, every power 0), goto/jog, and stop; the temporary
# lasers.conf also sets max_power 0. Nothing is cut. Still: clear the bed, hand
# near the E-stop, and know that the HEAD MOVES in most steps (a 40 mm square
# from the bed centre, at 20 mm/s).
#
# Steps, each asked for first (Enter = run, s = skip, q = quit):
#   a baseline          counters, status, settings dump (to a file)
#   b idle listen       does the controller push anything unprompted?
#   c reads mid-job     do reads answer while a dry run moves the head?
#   d second job        a dry run sent while the first is still running
#   e stop mid-job      stop during a dry run, then status
#   f bad file sum      dry run with a wrong sum: rejected or run anyway?
#   g chunk sweep       UDP only: packet size 200/500/1000/1400, timed
#   h position check    goto centre, jog +10 / -10 mm, goto back, status each
#   i unplug            you pull the cable during a watch
#   j lid open          you open the lid, then a dry run
#
# Everything goes to laser-edge-DATE/ (the directory is printed at the end):
# edge-cases.log with timestamps and exit codes, plus settings and watch files.
# Over serial only one process can hold the port, so step c cannot run a
# second watcher; it reads status right after the job starts instead.

set -u
here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/.." && pwd)
tool="$root/build/src/tools/ncad-laser"

usage="usage: $0 --serial DEV | --host IP"
case "${1:-}" in
    --serial) serial=1 ;;
    --host) serial=0 ;;
    *) echo "$usage" >&2; exit 2 ;;
esac
[ -n "${2:-}" ] || { echo "$usage" >&2; exit 2; }
link="$1 $2"
[ -x "$tool" ] || { echo "build first: cmake --build $root/build" >&2; exit 1; }

work=$(mktemp -d "${TMPDIR:-/tmp}/nlaser.XXXXXX")
cat > "$work/lasers.conf" <<'EOF'
[edge]
origin_corner = tr         # Black Cat: machine origin top-right
bed = 900 600
pointer_offset = 5.2 31.2
max_power = 0              # nothing can fire, even by mistake
ack_timeout_ms = 3000
EOF
# 20 mm/s: a 40 mm square (160 mm) takes about 8 s, long enough to interfere with.
cat > "$work/square.ljob" <<'EOF'
ljob 1 name="edge-square" source="hand-written"
origin 0.000 0.000
layer 1 color=1 mode=vector speed=20 power=10,10 passes=1 air=off order=1
path 1 closed
  0.000 0.000
  40.000 0.000
  40.000 40.000
  0.000 40.000
end
EOF
"$tool" plan "$work/square.ljob" -o "$work/square.planned.ljob" > /dev/null || exit 1
sq="$work/square.planned.ljob"
dev="--config $work/lasers.conf --device edge $link"

out="laser-edge-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$out" || exit 1
log="$out/edge-cases.log"

stamp() { while IFS= read -r l; do printf '%s %s\n' "$(date +%H:%M:%S)" "$l"; done; }
note() { printf '%s\n' "$*" | stamp | tee -a "$log"; }

# Run a command, show and log its output with timestamps, and log its exit code.
run() {
    note "\$ ${*#"$tool"}"
    { "$@" 2>&1; echo $? > "$work/rc"; } | stamp | tee -a "$log"
    note "  exit $(cat "$work/rc")"
}
# Milliseconds, for timing; perl ships with macOS and most Linux.
now_ms() { perl -MTime::HiRes=time -e 'printf "%d\n", time*1000' 2>/dev/null || echo $(( $(date +%s) * 1000 )); }
timed() {
    t0=$(now_ms)
    run "$@"
    note "  elapsed $(( $(now_ms) - t0 )) ms"
}
pause() { note "  (waiting $1 s)"; sleep "$1"; }

ask() {
    printf '\n== %s\n   %s\n   Enter = run, s = skip, q = quit: ' "$1" "$2"
    read -r answer
    case "$answer" in
        q*) note "== quit at: $1"; finish; exit 0 ;;
        s*) note "== $1: skipped"; return 1 ;;
    esac
    note "== $1"
    return 0
}

finish() {
    echo
    echo "Log directory: $(cd "$out" && pwd)"
    echo "Log file:      $(cd "$out" && pwd)/edge-cases.log"
    echo
    echo "Tell Claude (and attach the whole directory):"
    echo "  [ ] a  did counters/status/settings all answer? firmware string? settings file size?"
    echo "  [ ] b  did anything arrive unprompted while idle?"
    echo "  [ ] c  did reads answer while the head moved? latency? did state/progress change?"
    echo "  [ ] d  the second dry run: NAK / ERR / ignored / queued? did the head do both squares?"
    echo "  [ ] e  did the head stop at once? state afterwards?"
    echo "  [ ] f  bad sum: error reply, panel message, or traced anyway?"
    echo "  [ ] g  (UDP) which chunk sizes worked, upload times"
    echo "  [ ] h  did each goto/jog land where status says? which way did +x go on the machine?"
    echo "  [ ] i  what the tool printed when the cable came out; did it exit; did counters work again?"
    echo "  [ ] j  lid open: refused, error on the panel, or moved?"
    echo "  [ ] anything the panel showed or beeped that the log cannot know"
    rm -rf "$work"
}

centre() { run "$tool" goto 450 300 $dev; pause 3; }

note "edge cases $(date), link: $link, serial=$serial"
note "ncad-laser: $tool"
printf '\nThe laser cannot fire in this script. The head WILL move. Bed clear, hand near E-stop.\n'

step_a() {
    run "$tool" counters $dev
    run "$tool" status $dev
    note "\$ settings (to $out/settings.txt)"
    { "$tool" settings $dev > "$out/settings.txt" 2>&1; echo $? > "$work/rc"; }
    note "  exit $(cat "$work/rc"), $(wc -l < "$out/settings.txt" | tr -d ' ') lines"
    run "$tool" raw "DA 00 04 21" $dev
}
step_b() { run "$tool" watch --listen --duration 10 $dev; }
step_c() {
    centre
    if [ "$serial" = 1 ]; then
        note "serial: one process owns the port, so no background watch; status right after the job starts"
        run "$tool" dryrun "$sq" $dev
        run "$tool" status $dev
        pause 1
        run "$tool" status $dev
        pause 3
        run "$tool" status $dev
    else
        "$tool" watch --interval 200 --duration 16 --csv --local-port 40201 $dev > "$out/watch-during-dryrun.csv" 2>&1 &
        wp=$!
        pause 2
        run "$tool" dryrun "$sq" --wait $dev
        wait "$wp"
        note "watch (200 ms) during the dry run, saved to $out/watch-during-dryrun.csv:"
        stamp < "$out/watch-during-dryrun.csv" >> "$log"
        note "  $(grep -c ',ok$' "$out/watch-during-dryrun.csv") polls answered, $(grep -c ',none$' "$out/watch-during-dryrun.csv") not"
    fi
    pause 6
}
step_d() {
    centre
    run "$tool" dryrun "$sq" $dev
    pause 1
    run "$tool" dryrun "$sq" $dev
    run "$tool" status $dev
    pause 8
    run "$tool" status $dev
    note "if the head is still moving, press Enter at the next prompt only when it has stopped"
    pause 12
}
step_e() {
    centre
    run "$tool" dryrun "$sq" $dev
    pause 3
    run "$tool" stop $dev
    run "$tool" status $dev
    pause 2
    run "$tool" status $dev
}
step_f() {
    centre
    run "$tool" dryrun "$sq" --corrupt-sum $dev
    run "$tool" status $dev
    pause 3
    run "$tool" status $dev
    note "stopping whatever it is doing (harmless if idle)"
    run "$tool" stop $dev
}
step_g() {
    for n in 200 500 1000 1400; do
        note "-- chunk $n"
        centre
        timed "$tool" dryrun "$sq" --chunk "$n" $dev
        run "$tool" stop $dev
        pause 1
    done
}
step_h() {
    run "$tool" goto 450 300 $dev; pause 3; run "$tool" status $dev
    run "$tool" jog 10 0 $dev; pause 2; run "$tool" status $dev
    run "$tool" jog -10 0 $dev; pause 2; run "$tool" status $dev
    run "$tool" goto 450 300 $dev; pause 3; run "$tool" status $dev
}
step_i() {
    printf '\n   Starting a 30 s watch. PULL THE CABLE within 5 s of the first line.\n'
    printf '   (Over serial: the USB cable; over UDP: the Ethernet cable.)\n   Press Enter to start: '
    read -r _
    run "$tool" watch --interval 500 --duration 30 $dev
    printf '\n   Reconnect the cable now (serial: the device name may have changed: ls /dev/cu.usbserial*).\n'
    printf '   Enter when reconnected (type a new device path first if it changed): '
    read -r newdev
    if [ -n "$newdev" ] && [ "$serial" = 1 ]; then
        dev="--config $work/lasers.conf --device edge --serial $newdev"
        note "device is now $newdev"
    fi
    run "$tool" counters $dev
}
step_j() {
    printf '\n   OPEN THE LID NOW (the interlock), then press Enter: '
    read -r _
    note "lid open (operator says)"
    run "$tool" status $dev
    run "$tool" dryrun "$sq" $dev
    pause 2
    run "$tool" status $dev
    printf '   What did the panel show, and did the head move? Note it for Claude.\n'
    printf '   Then press Enter to send stop, and CLOSE THE LID: '
    read -r _
    run "$tool" stop $dev
}

ask "a. baseline (read-only)" "counters, status, settings dump, one raw read" && step_a
ask "b. listen idle 10 s (read-only)" "sends nothing; logs anything that arrives" && step_b
ask "c. reads during a dry run" "head to bed centre, dry run of a 40 mm square, status while it runs" && step_c
ask "d. second dry run while the first runs" "head to centre, two dry runs back to back" && step_d
ask "e. stop mid-dry-run" "dry run, stop after 3 s, status" && step_e
ask "f. dry run with a wrong file sum" "laser-off; does the controller reject or trace it?" && step_f
if [ "$serial" = 1 ]; then
    note "== g. chunk sweep: skipped (UDP only; serial has no packets)"
else
    ask "g. packet size sweep (UDP)" "dry run at 200/500/1000/1400 byte packets, timed, stop after each" && step_g
fi
ask "h. position check" "goto centre, jog +10, jog -10, goto centre, status after each" && step_h
ask "i. unplug test" "YOU pull the cable during a 30 s watch" && step_i
ask "j. lid open" "YOU open the lid, then a dry run" && step_j

note ""
note "done."
finish
