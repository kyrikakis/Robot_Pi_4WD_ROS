#!/bin/bash
# Announce the Pi as rainbow.local too (its hostname stays raspberrypi), so the
# phone remote is at http://rainbow.local/. Announces the Wi-Fi's IPv4 address
# and the global IPv6 one if there is one (with only an A record, clients that
# ask for both waited 5 s for the AAAA answer). When an address changes, the
# announcements restart with the new ones.
# Installed on the Pi host by pi-files/rainbow-mdns.service (see the README).

NAME="${1:-rainbow.local}"
pids=()
current=""

stop_all() {
    for p in "${pids[@]}"; do kill "$p" 2>/dev/null; done
    pids=()
}
trap 'stop_all; exit 0' TERM INT

src_of() {
    ip "$1" route get "$2" 2>/dev/null | awk '{for (i = 1; i < NF; i++) if ($i == "src") print $(i + 1)}'
}

while true; do
    ip4=$(src_of -4 1.1.1.1)
    ip6=$(src_of -6 2606:4700:4700::1111)
    alive=1
    for p in "${pids[@]}"; do kill -0 "$p" 2>/dev/null || alive=0; done
    if [ -n "$ip4" ] && { [ "$ip4 $ip6" != "$current" ] || [ "$alive" = 0 ]; }; then
        stop_all
        avahi-publish -a -R "$NAME" "$ip4" &
        pids+=($!)
        if [ -n "$ip6" ]; then
            avahi-publish -a -R "$NAME" "$ip6" &
            pids+=($!)
        fi
        current="$ip4 $ip6"
        echo "announcing $NAME at $current"
    fi
    sleep 30
done
