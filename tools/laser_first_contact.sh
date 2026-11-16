#!/bin/sh
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, Sadie Forbes
#
# First contact with a Ruida laser, one step at a time, everything logged.
#
#   tools/laser_first_contact.sh --serial /dev/cu.usbserial-XXXX
#   tools/laser_first_contact.sh --host 192.168.1.100
#
# Steps, each asked for before it runs (Enter = run, s = skip, q = quit):
#   1 counters   read-only: proves the link, the magic and the reply format
#   2 status     read-only: state, head position, bed size
#   3 frame      LASER OFF: traces the card test's 30 mm box from the head
#   4 send       the card test at 10 % power, 100 mm/s (asks 'yes' again)
#   5 status     again, after the job
#
# The log (first-contact-DATE.log, in the current directory) is what to bring
# back. Settings for the Black Cats: machine origin top-right, and send
# refuses anything above 20 % power while testing.
#
# USB: the controller's PC port is usually USB-B (a printer cable from the
# laptop's USB-A, or USB-C to USB-B); `ls /dev/cu.usbserial*` should show it.
# Ethernet: give the laptop a static address on the controller's subnet
# (controller's IP is on its panel; laptop e.g. same first three numbers,
# last one different, mask 255.255.255.0). A direct cable is fine. Only one
# program can talk to the controller over UDP at a time: quit LightBurn.

set -u
here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/.." && pwd)
tool="$root/build/src/tools/ncad-laser"

case "${1:-}" in
    --serial|--host) [ -n "${2:-}" ] || { echo "usage: $0 --serial DEV | --host IP" >&2; exit 2; } ;;
    *) echo "usage: $0 --serial DEV | --host IP" >&2; exit 2 ;;
esac
link="$1 $2"
[ -x "$tool" ] || { echo "build first: cmake --build $root/build" >&2; exit 1; }

work=$(mktemp -d "${TMPDIR:-/tmp}/nlaser.XXXXXX")
cat > "$work/lasers.conf" <<'EOF'
[first-contact]
origin_corner = tr     # Black Cat: machine origin top-right
max_power = 20         # send refuses any layer above this
ack_timeout_ms = 3000
EOF
dev="--config $work/lasers.conf --device first-contact $link"
"$tool" plan "$root/examples/card_test.ljob" -o "$work/card.ljob" > /dev/null || exit 1

log="first-contact-$(date +%Y%m%d-%H%M%S).log"
note() { printf '%s\n' "$*" | tee -a "$log"; }
step() {
    n=$1; title=$2; shift 2
    printf '\n== %s. %s\n   %s\n   Enter = run, s = skip, q = quit: ' "$n" "$title" "$*"
    read -r answer
    case "$answer" in
        q*) note "== quit at step $n"; exit 0 ;;
        s*) note "== $n. $title: skipped"; return ;;
    esac
    note "== $n. $title: $*"
    "$@" 2>&1 | tee -a "$log"
    note "   (exit status: see above; $(date +%H:%M:%S))"
}

note "first contact $(date), link: $link"
note "ncad-laser: $tool"

step 1 "counters (read-only)" "$tool" counters $dev
step 2 "status (read-only)" "$tool" status $dev
printf '\nBefore step 3: card on the bed, head over the MIDDLE of the card.\n'
printf 'The box will be traced from the head; the laser does not fire.\n'
step 3 "frame, laser off" "$tool" frame "$work/card.ljob" $dev
printf '\nBefore step 4: same position. Hand near Stop. 10 %% power, 100 mm/s.\n'
printf 'The circle cuts first, then the square. ncad-laser will ask for yes.\n'
step 4 "send the card test" "$tool" send "$work/card.ljob" $dev
step 5 "status after the job" "$tool" status $dev

note ""
note "done. Bring back $log, and note: did the panel show the job; which way"
note "did the frame and the cut go from the head; did it look like 10 % power?"
rm -rf "$work"
