#!/usr/bin/env bash
exec bash "$(cd "$(dirname "$0")/.." && pwd)/tools/hardware/power/maxn_lock.sh" "$@"
