#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "${ROOT_DIR}"

PATTERNS=(
  'BUS_SERVO\\b|0x06\\b'
  '\\bIO\\b[^/]|0x04\\b'
  '\\b0x90\\b|\\b0x91\\b|\\b0x99\\b'
  '\\b0xA0\\b|\\b0xA1\\b'
  '\\bRRCV1_|RRC_FUNC_LEGACY|RRC_PROTO_COMPAT_LEGACY'
)

EXIT=0

for pattern in "${PATTERNS[@]}"; do
  if matches=$(rg --color=never --line-number --no-heading \
      --glob '!Third_Party/**' --glob '!docs/**' \
      --glob '!scripts/check_no_legacy.sh' \
      --glob '!scripts/check_protocol2_promotion.sh' \
      --glob '!Core/Src/rrclite_legacy_shim.c' \
      -- "$pattern" .); then
    printf 'Legacy pattern "%s" matches:\n' "$pattern" >&2
    printf '%s\n' "$matches" >&2
    EXIT=1
  fi
done

if [[ ${EXIT} -ne 0 ]]; then
  echo 'Legacy identifiers found. Please migrate remaining references to the v2 IDs.' >&2
  exit ${EXIT}
fi

echo 'No legacy identifiers detected.'
