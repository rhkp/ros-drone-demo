#!/usr/bin/env bash
set -euo pipefail

# Run this on the validated build VM. Keep versions.env local and ignored;
# versions.env.example is the only configuration intended for Git.
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
version_file="${VERSION_FILE:-${repo_dir}/versions.env}"
if [[ ! -f "${version_file}" ]]; then
  echo "Missing ${version_file}; copy versions.env.example to versions.env on the build VM." >&2
  exit 1
fi
source "${version_file}"

: "${REGISTRY:?REGISTRY is required}"
: "${RELEASE_TAG:?RELEASE_TAG is required}"
engine="${BUILD_ENGINE:-podman}"

build_one() {
  local name="$1"
  local tag="${REGISTRY}/hbr-drone-${name}:${RELEASE_TAG}"
  local -a build_args=(--build-arg "REGISTRY=${REGISTRY}" --build-arg "BASE_TAG=${BASE_TAG:-v0.1.0}")
  if [[ "${name}" == observer && -n "${BOOTC_OS_IMAGE:-}" ]]; then
    build_args+=(--build-arg "BOOTC_OS_IMAGE=${BOOTC_OS_IMAGE}")
  fi
  "${engine}" build "${build_args[@]}" \
    --file "${repo_dir}/images/drone-${name}/Containerfile" \
    --tag "${tag}" "${repo_dir}"
  echo "Built ${tag}"
}

case "${1:-all}" in
  world) build_one world ;;
  observer) build_one observer ;;
  all) build_one world; build_one observer ;;
  *) echo "Usage: $0 [world|observer|all]" >&2; exit 2 ;;
esac
