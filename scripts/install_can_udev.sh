#!/usr/bin/env bash
exec bash "$(cd "$(dirname "$0")/.." && pwd)/tools/hardware/can/install_can_udev.sh" "$@"
