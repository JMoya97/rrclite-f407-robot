#!/usr/bin/env bash
set -euo pipefail

REMOTE_URL="https://github.com/JMoya97/rrclite-f407-robot.git"

# Ensure we're in a git repo
git rev-parse --git-dir >/dev/null 2>&1 || { echo "Not a git repo"; exit 1; }

# Add 'origin' if the container doesn't have it yet
if ! git remote get-url origin >/dev/null 2>&1; then
  echo "Adding remote 'origin' -> $REMOTE_URL"
  git remote add origin "$REMOTE_URL"
fi

# Optional: set upstream for the current branch if it exists on origin
branch="$(git rev-parse --abbrev-ref HEAD || echo main)"
git fetch origin || true
git rev-parse --verify "origin/$branch" >/dev/null 2>&1 && \
  git branch --set-upstream-to="origin/$branch" "$branch" || true

# Only clean in CI to avoid nuking local work
if [ "${CI:-}" = "true" ]; then
  git reset --hard HEAD
  git clean -fdx
fi

echo "setup: done; no tracked files were modified."

# Guard to ensure setup didn't dirty tracked files
if ! git diff --quiet --ignore-submodules -- .; then
  echo "Error: setup modified tracked files."
  git status --porcelain
  exit 1
fi
