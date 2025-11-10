.RECIPEPREFIX := >
.DEFAULT_GOAL := help

.PHONY: help
help:
>@echo "Available developer targets:"
>@echo "  make check-no-legacy"
>@echo "    - Run protocol legacy identifier guard checks."
>@echo "  make check-protocol2"
>@echo "    - Ensure no legacy IDs/shims remain after Protocol 2 promotion."
>@echo "  make check-runtime-health"
>@echo "    - Check that SYS diagnostics handlers are present in dispatch tables."

.PHONY: check-no-legacy
check-no-legacy:
>./scripts/check_no_legacy.sh

.PHONY: check-protocol2
check-protocol2:
>./scripts/check_protocol2_promotion.sh

.PHONY: check-runtime-health
check-runtime-health:
>./scripts/check_runtime_health.sh
