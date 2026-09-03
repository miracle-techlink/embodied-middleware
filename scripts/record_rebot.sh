#!/usr/bin/env bash
exec bash "$(cd "$(dirname "$0")/.." && pwd)/tools/acquisition/record_rebot.sh" "$@"
