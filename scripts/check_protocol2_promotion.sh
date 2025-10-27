#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "${ROOT_DIR}"

readonly -a EXCLUDES=(
  '--glob' '!Third_Party/**'
  '--glob' '!docs/**'
  '--glob' '!scripts/check_no_legacy.sh'
  '--glob' '!scripts/check_protocol2_promotion.sh'
)

declare -a PATTERNS=(
  'BUS_SERVO\\b|0x06\\b:BUS_SERVO / 0x06 legacy identifiers'
  '\\bIO\\b[^/]|0x04\\b:IO(0x04) legacy identifiers'
  '\\b0x90\\b|\\b0x91\\b|\\b0x99\\b:Encoder legacy subs (0x90/0x91/0x99)'
  '\\b0xA0\\b|\\b0xA1\\b:SYS battery legacy subs (0xA0/0xA1)'
  'IMU[^\n]*0x07:IMU legacy function id (0x07)'
  'RRC_FUNC_.*_LEGACY:Legacy function constants'
  'rrc_legacy_translate_rx:Legacy RX shim'
  'rrc_map_outbound:Legacy TX shim'
  'RRC_PROTO_COMPAT_LEGACY:Legacy compatibility flag'
  'g_rrc_wire_mode:Legacy wire-mode state'
  '\\bwire_mode\\b:Legacy wire-mode helpers'
  'RRCV2_FUNC_:Deprecated v2 prefix'
)

EXIT_CODE=0

run_search() {
  local pattern_with_label="$1"
  local pattern="${pattern_with_label%%:*}"
  local label="${pattern_with_label#*:}"

  if matches=$(rg --color=never --line-number --no-heading \
      "${EXCLUDES[@]}" -- "${pattern}" .); then
    printf 'Protocol 2 promotion guard tripped (%s):\n' "${label}" >&2
    printf '%s\n' "${matches}" >&2
    EXIT_CODE=1
  fi
}

for entry in "${PATTERNS[@]}"; do
  run_search "${entry}"
done

if [[ ${EXIT_CODE} -ne 0 ]]; then
  echo 'Protocol 2 promotion check FAILED. See matches above.' >&2
  exit "${EXIT_CODE}"
fi

echo 'Protocol 2 promotion check passed.'
