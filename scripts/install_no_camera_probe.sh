#!/usr/bin/env bash
exec bash "$(cd "$(dirname "$0")/.." && pwd)/tools/hardware/usb/install_no_camera_probe.sh" "$@"
