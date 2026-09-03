#!/usr/bin/env bash
  set -euo pipefail

  cd "$(git rev-parse --show-toplevel)"

  if [[ -n "$(git status --porcelain)" ]]; then
      echo "Update stopped: commit or stash your local changes first."
      exit 1
  fi

  git switch plp-machine-config
  git fetch origin
  git merge --no-edit origin/fair-2026

  echo "PLP machine branch updated successfully."
