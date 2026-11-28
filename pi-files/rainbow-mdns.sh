#!/bin/bash
# Announce the Pi as rainbow.local too (its hostname stays raspberrypi), so the
# phone remote is at http://rainbow.local/. Follows the Wi-Fi address: when it
# changes, the announcement is restarted with the new one.
# Installed on the Pi host by pi-files/rainbow-mdns.service (see the README).

NAME="${1:-rainbow.local}"
pid=""
current=""
trap '[ -n "$pid" ] && kill "$pid"; exit 0' TERM INT

while true; do
    ip=$(ip -4 route get 1.1.1.1 2>/dev/null | awk '{for (i = 1; i < NF; i++) if ($i == "src") print $(i + 1)}')
    if [ -n "$pid" ] && ! kill -0 "$pid" 2>/dev/null; then
        pid=""
    fi
    if [ -n "$ip" ] && { [ "$ip" != "$current" ] || [ -z "$pid" ]; }; then
        [ -n "$pid" ] && kill "$pid" 2>/dev/null
        avahi-publish -a -R "$NAME" "$ip" &
        pid=$!
        current="$ip"
        echo "announcing $NAME at $ip"
    fi
    sleep 30
done
