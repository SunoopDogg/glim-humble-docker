#!/usr/bin/env bash
# Session-start network setup for the Go2 on the onboard eno1 NIC. Run INSIDE the
# privileged glim container (host netns). Runtime-only — resets on reboot, re-run each
# session. Idempotent. The Ouster shares eno1 on its own subnet (see jetson-nav-run-order).
set -uo pipefail
IFACE="${GO2_IFACE:-eno1}"
GO2_HOST_IP="${GO2_HOST_IP:-192.168.123.222/24}"   # host addr on Go2 subnet (Go2 at .161)

ip addr add "$GO2_HOST_IP" dev "$IFACE" 2>/dev/null \
  && echo "added $GO2_HOST_IP on $IFACE" || echo "$GO2_HOST_IP already on $IFACE (ok)"
ip route replace 224.0.0.0/4 dev "$IFACE"           # DDS SPDP multicast egress
sysctl -w net.ipv4.conf."$IFACE".rp_filter=0        # link-local / multi-subnet RX
sysctl -w net.ipv4.conf.all.rp_filter=0
ip route flush cache

# UFW is the real Go2-discovery blocker on this Jetson (host INPUT policy DROP drops
# the Go2 DDS multicast 239.255.0.1). The fix is a ONE-TIME HOST command (persists):
#   sudo ufw allow in on eno1 ; sudo ufw allow 7502/udp ; sudo ufw allow 7503/udp
# This container script cannot run host ufw; warn if it looks un-opened.
if command -v ufw >/dev/null 2>&1 && ufw status 2>/dev/null | grep -qi "Status: active"; then
  ufw status 2>/dev/null | grep -qiE "Anywhere on $IFACE|$IFACE " \
    || echo "WARN: ufw active but no rule for $IFACE — run on HOST: sudo ufw allow in on $IFACE"
fi
echo "net_setup_go2: $IFACE ready (mcast route + rp_filter=0)"
