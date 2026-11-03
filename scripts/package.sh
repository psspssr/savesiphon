#!/usr/bin/env bash
# Usage: scripts/package.sh <ap_core.rbf> <version> <outdir>
set -euo pipefail
rbf=$1; version=$2; out=$3
root=$(cd "$(dirname "$0")/.." && pwd)
stage=$(mktemp -d)
cp -R "$root/pkg/." "$stage/"
python3 "$root/scripts/reverse_bits.py" "$rbf" "$stage/Cores/psspssr.SaveSiphon/bitstream.rbf_r"
# stamp version/date into core.json
python3 - "$stage/Cores/psspssr.SaveSiphon/core.json" "$version" <<'PY'
import json, sys, datetime
p, v = sys.argv[1], sys.argv[2]
d = json.load(open(p))
d["core"]["metadata"]["version"] = v
d["core"]["metadata"]["date_release"] = datetime.date.today().isoformat()
json.dump(d, open(p, "w"), indent=4)
PY
mkdir -p "$out"
zip_path="$(cd "$out" && pwd)/psspssr.SaveSiphon_${version}.zip"
(cd "$stage" && zip -r -X "$zip_path" Assets Cores Platforms >/dev/null)
echo "$zip_path"
