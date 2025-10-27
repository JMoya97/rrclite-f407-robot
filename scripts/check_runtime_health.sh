#!/bin/sh

set -eu

ROOT="$(CDPATH= cd -- "$(dirname "$0")/.." && pwd)"

TABLE_FILE="$ROOT/Core/Src/rrclite_packets.c"

missing=0

check_entry() {
  symbol="$1"
  if ! grep -q "$symbol" "$TABLE_FILE"; then
    printf 'Missing SYS sub %s in %s\n' "$symbol" "$TABLE_FILE" >&2
    missing=1
  fi
}

check_entry "RRC_SYS_STATS_GET"
check_entry "RRC_SYS_HEALTH_GET"
check_entry "RRC_SYS_SELFTEST_GET"

exit "$missing"
