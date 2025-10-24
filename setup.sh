#!/usr/bin/env bash
set -euo pipefail

# Ensure the tree is clean before the grader applies its patch
git reset --hard HEAD
git clean -fdx

echo "setup: nothing to do; not mutating tracked files."

# Guard: fail if anything changed (it shouldn't)
if ! git diff --quiet --ignore-submodules -- .; then
  echo "Error: setup modified tracked files."
  git status --porcelain
  exit 1
fi
