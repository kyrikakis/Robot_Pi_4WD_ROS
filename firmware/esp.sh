#!/usr/bin/env bash
# ESP board tools, run on the Pi host (the board is on its USB serial port).
#
#   ./esp.sh flash [build_dir]   write a build (default ./build, from build.sh)
#   ./esp.sh backup [file]       read the whole 4 MB flash to a file
#   ./esp.sh restore <file>      write a whole-flash image back (e.g. the stock backup)
#   ./esp.sh info                chip, flash and MAC
#
# The micro-ROS agent (supervisor program rainbow_microros in the robot container)
# owns the port: it is stopped first and started again afterwards. Our firmware
# reconnects to it by itself; the stock firmware needs a board reset after an agent
# restart, which esptool's reset after writing provides.
set -euo pipefail
cd "$(dirname "$0")"

PORT=${ESP_PORT:-/dev/ttyUSB_ROS}
BAUD=${ESP_BAUD:-460800}
VENV=${ESPTOOL_VENV:-$HOME/esptool-venv}
ESPTOOL_VERSION=4.8.1
BITSTRING_VERSION=4.2.3  # espefuse breaks with bitstring 4.3

if [ ! -x "$VENV/bin/esptool.py" ]; then
  python3 -m venv "$VENV"
  "$VENV/bin/pip" install -q "esptool==$ESPTOOL_VERSION" "bitstring==$BITSTRING_VERSION"
fi
ESPTOOL=("$VENV/bin/esptool.py" --chip esp32s3 -p "$PORT" -b "$BAUD")

agent() {  # stop | start
  docker exec robot supervisorctl "$1" rainbow_microros >/dev/null 2>&1 || true
}

cmd=${1:-}
case "$cmd" in
  flash)
    dir=${2:-build}
    [ -f "$dir/flash_args" ] || { echo "no $dir/flash_args: run build.sh first" >&2; exit 1; }
    agent stop; trap 'agent start' EXIT
    (cd "$dir" && "${ESPTOOL[@]}" --before default_reset --after hard_reset write_flash @flash_args)
    ;;
  backup)
    file=${2:-esp-flash-$(date +%Y%m%d-%H%M%S).bin}
    agent stop; trap 'agent start' EXIT
    "${ESPTOOL[@]}" read_flash 0 0x400000 "$file"
    sha256sum "$file"
    ;;
  restore)
    file=${2:?image file}
    [ "$(stat -c %s "$file")" = 4194304 ] || { echo "$file is not a 4 MB image" >&2; exit 1; }
    agent stop; trap 'agent start' EXIT
    "${ESPTOOL[@]}" --before default_reset --after hard_reset write_flash 0x0 "$file"
    ;;
  info)
    agent stop; trap 'agent start' EXIT
    "${ESPTOOL[@]}" flash_id
    ;;
  *)
    sed -n '2,13p' "$0"; exit 1 ;;
esac
