#!/usr/bin/env bash
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="$HERE/src/middleware:${PYTHONPATH:-}"
exec python3 "$HERE/admin/rig_doctor.py" "$@"
