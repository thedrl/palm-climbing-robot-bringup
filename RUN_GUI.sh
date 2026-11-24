#!/usr/bin/env bash
set -eu
revision_dir="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
exec /home/frc-orin/miniforge3/bin/python3 -B "$revision_dir/Gui/climber_gui5.py"
