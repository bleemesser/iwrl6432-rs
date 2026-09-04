#!/usr/bin/env bash
# Run a DSS script from this directory, e.g.:
#   dss/run.sh load_fw.js
# Uses the ccs_base tree vendored at ../ti/ccs_base (TI's Debug Server
# Scripting, copied in from UniFlash/CCS).
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export FW_ROOT="${FW_ROOT:-$(cd "$DIR/.." && pwd)}"
DSS_SH="${DSS_SH:-$FW_ROOT/ti/ccs_base/scripting/bin/dss.sh}"

[ -x "$DSS_SH" ] || { echo "dss.sh not found at $DSS_SH (set DSS_SH)"; exit 1; }
exec "$DSS_SH" "$DIR/$1" "${@:2}"
