.DEFAULT_GOAL := help

.PHONY: help
help:
	@echo "Available developer targets:"
	@echo "  make check-no-legacy"
	@echo "    - Run protocol legacy identifier guard checks."

.PHONY: check-no-legacy
check-no-legacy:
	./scripts/check_no_legacy.sh
