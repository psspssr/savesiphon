#!/usr/bin/env bash
# Runs the system testbench for every scenario (or the ones given as args).
#   sim/run_all.sh            all scenarios
#   sim/run_all.sh 3 13       just those
set -euo pipefail
cd "$(dirname "$0")/.."
SRC="src/fpga/core/siphon_ctrl.v src/fpga/core/gba_cart_bus.v src/fpga/core/gb_cart_bus.v src/fpga/core/cart_write_guard.v"
LIST=${*:-"0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17"}
out=$(mktemp -d)
fail=0
for t in $LIST; do
  le=1; [ "$t" = 17 ] && le=0
  iverilog -g2005 -DCT=$t -DAPF_LE=$le -o "$out/tb$t" sim/tb_siphon.v $SRC
  LC_ALL=C perl -e 'alarm 1800; exec @ARGV' vvp -n "$out/tb$t" > "$out/log$t" 2>&1 &
done
wait
for t in $LIST; do
  grep -h "PASS\|FAIL\|ERROR" "$out/log$t" | head -6
  grep -q "^PASS" "$out/log$t" || { fail=1; echo "--- log $t: $out/log$t"; }
done
exit $fail
