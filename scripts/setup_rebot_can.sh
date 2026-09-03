#!/usr/bin/env bash
exec bash "$(cd "$(dirname "$0")/.." && pwd)/tools/hardware/can/setup_rebot_can.sh" "$@"
