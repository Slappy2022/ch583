#!/bin/bash
set -eux

readonly BASE_DIR="$(
  cd -P "$(dirname "${BASH_SOURCE[0]}/..")"
  pwd
)"
readonly EXAM_DIR="/ch583/EVT/EXAM"

readonly TAG=ch583-build
readonly DIR=.

main() {
  local -r app=$1

  mkdir -p "${BASE_DIR}/"Build
  docker kill $(docker ps -q) || true
  docker build -t "${TAG}" "${DIR}"
  docker run -i \
    -v "${BASE_DIR}/"Build:"${EXAM_DIR}"/Build \
    --workdir "${EXAM_DIR}/$1" \
    -t "${TAG}" make clean all
}

main "$@"
