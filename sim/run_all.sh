#!/usr/bin/env bash
# Runs the cartridge-model testbench for every save type.
set -euo pipefail
cd "$(dirname "$0")/.."
out=$(mktemp -d)
fail=0
for t in 0 1 2 3 4 5 6 7 8 9 10; do
  iverilog -g2005 -DCART_TYPE=$t -o "$out/tb$t" sim/tb_siphon.v \
    src/fpga/core/siphon_ctrl.v src/fpga/core/gba_cart_bus.v src/fpga/core/cart_write_guard.v
  vvp -n "$out/tb$t" > "$out/log$t" &
done
wait
for t in 0 1 2 3 4 5 6 7 8 9 10; do
  grep -h "PASS\|FAIL\|ERROR" "$out/log$t" | head -5
  grep -q "^PASS" "$out/log$t" || fail=1
done
exit $fail
