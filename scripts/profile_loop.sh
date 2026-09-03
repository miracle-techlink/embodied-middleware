#!/usr/bin/env bash
exec bash "$(cd "$(dirname "$0")/.." && pwd)/tools/diagnostics/profile_loop.sh" "$@"
