#!/usr/bin/env bash
exec bash "$(cd "$(dirname "$0")/.." && pwd)/tools/acquisition/teleop_rebot.sh" "$@"
