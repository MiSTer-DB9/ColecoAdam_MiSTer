#!/bin/sh
# Boot the simulated ADAM against the native FujiNet-PC ADAM build, over Bus-over-IP.
#
#   ./fujinet_test.sh OUTDIR [Vemu options...]
#
# Starts the simulator listening on TCP 65216 (--fujinet, no local media), then
# FujiNet-PC, which connects to it, waits for the simulator to finish and stops
# FujiNet-PC. OUTDIR gets sim.log, fujinet.log, bytes.log (every byte on the wire,
# with the frame number) and whatever frames --shots/--every ask for.
#
# FUJI_DIST is FujiNet-PC's dist directory (the binary and its data/), FUJI_RUN
# holds fnconfig.ini (with [BOIP] enabled=1 port=65216 and the dummy WiFi on) and
# the SD folder it serves as host 1. MULTI_PLATFORM.md in FujiNet_MiSTer has the
# native build recipe.

FUJI_DIST=${FUJI_DIST:-$HOME/dev2/FujiNet_MiSTer/build_host/dist}
FUJI_RUN=${FUJI_RUN:-$HOME/dev2/FujiNet_MiSTer/build_host/run}

out=$1
[ -z "$out" ] && { echo "usage: $0 OUTDIR [Vemu options]"; exit 1; }
shift
mkdir -p "$out"

${VEMU:-./obj_dir/Vemu} --headless --adam --no-media --fujinet --outdir "$out" \
    --fujinet-log "$out/bytes.log" "$@" > "$out/sim.log" 2>&1 &
sim=$!
sleep 1
(cd "$FUJI_DIST" && exec ./fujinet -c "$FUJI_RUN/fnconfig.ini" -s "$FUJI_RUN/SD") > "$out/fujinet.log" 2>&1 &
fn=$!

wait $sim
rc=$?
kill $fn 2>/dev/null
wait $fn 2>/dev/null
grep 'FujiNet' "$out/sim.log" | tail -n 12
tail -n 2 "$out/sim.log"
exit $rc
