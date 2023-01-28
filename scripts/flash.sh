#!/bin/bash
set -eux

readonly BASE_DIR="$(
  cd -P "$(dirname "${BASH_SOURCE[0]}/..")"
  pwd
)"

main() {
  local -r app=$1
  "${BASE_DIR}"/scripts/build.sh "${app}"
  wchisp flash "${BASE_DIR}/Build/$1.elf"
}

main "$@"
