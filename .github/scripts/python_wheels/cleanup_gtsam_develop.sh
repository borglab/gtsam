#!/usr/bin/env bash

# This script deletes old releases from the gtsam-develop project on PyPI.
# The cleanup account must have owner privileges on the project.

set -euo pipefail

PACKAGE="gtsam-develop"
DEFAULT_KEEP=5
KEEP="$DEFAULT_KEEP"
PYPI_USER="${PYPI_CLEANUP_USERNAME:-}"
ASSUME_YES=0
DRY_RUN=0

usage() {
  cat <<EOF
Usage: $(basename "$0") [options] [pypi_username]

Deletes old releases from the ${PACKAGE} project on PyPI while keeping the most
recent releases by PyPI upload time. THIS OPERATION IS PERMANENT unless
--dry-run is used.

Options
  -k, --keep N   Number of latest releases to keep (default: ${DEFAULT_KEEP})
  -y, --yes      Do not prompt before deleting
      --dry-run  Query PyPI and validate the matching releases without deleting
  -h, --help     Show this help

Environment
  PYPI_CLEANUP_USERNAME  PyPI user name used when no positional user is supplied
  PYPI_CLEANUP_PASSWORD  PyPI password read by pypi-cleanup for noninteractive use

Examples
  $ $(basename "$0") --dry-run
  $ $(basename "$0") --keep 5 yambati3
  $ PYPI_CLEANUP_USERNAME=yambati3 PYPI_CLEANUP_PASSWORD=... $(basename "$0") --yes
EOF
}

die() {
  echo "error: $*" >&2
  exit 1
}

POSITIONAL_USER=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    -k|--keep)
      option="$1"
      shift
      [[ $# -gt 0 ]] || die "$option requires a value"
      KEEP="$1"
      ;;
    --keep=*)
      KEEP="${1#*=}"
      ;;
    -y|--yes)
      ASSUME_YES=1
      ;;
    --dry-run)
      DRY_RUN=1
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    -*)
      usage
      die "unknown option: $1"
      ;;
    *)
      [[ -z "$POSITIONAL_USER" ]] || die "unexpected extra argument: $1"
      POSITIONAL_USER="$1"
      ;;
  esac
  shift
done

if [[ -n "$POSITIONAL_USER" ]]; then
  PYPI_USER="$POSITIONAL_USER"
fi

[[ "$KEEP" =~ ^[0-9]+$ ]] || die "--keep must be a positive integer"
(( KEEP >= 1 )) || die "--keep must be at least 1"

python3 - <<'PY' >/dev/null 2>&1 || die "pypi-cleanup is required; install it with: python3 -m pip install pypi-cleanup"
import pypi_cleanup  # noqa: F401
PY

CANDIDATES_FILE=$(mktemp)
trap 'rm -f "$CANDIDATES_FILE"' EXIT

python3 - "$PACKAGE" "$KEEP" "$CANDIDATES_FILE" <<'PY'
import datetime as dt
import json
import re
import sys
import urllib.error
import urllib.request

package, keep_text, output_path = sys.argv[1:4]
keep = int(keep_text)


def parse_upload_time(file_info):
    value = file_info.get("upload-time") or file_info.get("upload-time-iso-8601")
    if not value:
        return None
    if value.endswith("Z"):
        value = value[:-1] + "+00:00"
    uploaded = dt.datetime.fromisoformat(value)
    if uploaded.tzinfo is None:
        uploaded = uploaded.replace(tzinfo=dt.timezone.utc)
    return uploaded


def package_matches_file(package_name, version, file_info):
    filename = file_info["filename"].lower()
    normalized_package = package_name.lower().replace("-", "_")
    dashed_package = package_name.lower()
    version = version.lower()

    if filename.endswith((".whl", ".egg", ".src.rpm")):
        return filename.startswith(f"{normalized_package}-{version}-")

    return filename in {
        f"{dashed_package}-{version}.tar.gz",
        f"{dashed_package}-{version}.zip",
        f"{normalized_package}-{version}.tar.gz",
        f"{normalized_package}-{version}.zip",
    }


request = urllib.request.Request(
    f"https://pypi.org/simple/{package}/",
    headers={"Accept": "application/vnd.pypi.simple.v1+json"},
)

try:
    with urllib.request.urlopen(request, timeout=30) as response:
        project_info = json.load(response)
except urllib.error.HTTPError as error:
    raise SystemExit(f"Unable to query {package!r} from PyPI: HTTP {error.code}") from error
except urllib.error.URLError as error:
    raise SystemExit(f"Unable to query {package!r} from PyPI: {error.reason}") from error

release_upload_times = {}
files = project_info.get("files", [])
for version in project_info.get("versions", []):
    upload_times = [
        uploaded
        for file_info in files
        if package_matches_file(package, version, file_info)
        for uploaded in [parse_upload_time(file_info)]
        if uploaded is not None
    ]
    if upload_times:
        release_upload_times[version] = max(upload_times)

releases = sorted(
    release_upload_times.items(),
    key=lambda item: (item[1], item[0]),
    reverse=True,
)
kept_releases = releases[:keep]
delete_releases = releases[keep:]

print(f"Found {len(releases)} release(s) for {package!r}.")
print(f"Keeping the latest {len(kept_releases)} release(s):")
for version, uploaded in kept_releases:
    print(f"  {version} ({uploaded.isoformat()})")

if delete_releases:
    print(f"Deleting {len(delete_releases)} older release(s):")
    for version, uploaded in delete_releases:
        print(f"  {version} ({uploaded.isoformat()})")
else:
    print(f"No deletion needed; release count is not greater than {keep}.")

with open(output_path, "w", encoding="utf-8") as output:
    for version, _ in delete_releases:
        output.write(f"{version}\t^{re.escape(version)}$\n")
PY

DELETE_VERSIONS=()
CLEANUP_ARGS=(-p "$PACKAGE")
while IFS=$'\t' read -r version pattern; do
  [[ -n "$version" ]] || continue
  DELETE_VERSIONS+=("$version")
  CLEANUP_ARGS+=(-r "$pattern")
done < "$CANDIDATES_FILE"

if (( ${#DELETE_VERSIONS[@]} == 0 )); then
  echo "Done."
  exit 0
fi

if (( DRY_RUN )); then
  echo "Running pypi_cleanup in query-only mode..."
  python3 -m pypi_cleanup.__init__ "${CLEANUP_ARGS[@]}" --query-only
  echo "Dry run complete."
  exit 0
fi

if [[ -z "$PYPI_USER" ]]; then
  if [[ -t 0 ]]; then
    read -rp "Enter your PyPI user name: " PYPI_USER
  else
    die "no PyPI user name supplied; pass one explicitly or set PYPI_CLEANUP_USERNAME"
  fi
fi
[[ -n "$PYPI_USER" ]] || die "no PyPI user name supplied"

if [[ -z "${PYPI_CLEANUP_PASSWORD:-}" && ! -t 0 ]]; then
  die "PYPI_CLEANUP_PASSWORD must be set for noninteractive cleanup"
fi

echo "-----------------------------------------------------------------------"
echo "WARNING: This WILL permanently delete ${#DELETE_VERSIONS[@]} old release(s)"
echo "         of 'gtsam-develop' on PyPI for user '$PYPI_USER'."
echo "         The latest $KEEP release(s) will be kept."
echo "         This cannot be undone."
echo "-----------------------------------------------------------------------"
printf 'Releases selected for deletion:\n'
printf '  %s\n' "${DELETE_VERSIONS[@]}"

if (( ! ASSUME_YES )); then
  read -rp "Proceed? [y/N]: " REPLY
  REPLY=$(echo "$REPLY" | tr '[:upper:]' '[:lower:]')
  [[ "$REPLY" == "y" || "$REPLY" == "yes" ]] || { echo "Aborted."; exit 0; }
fi

echo "Running pypi_cleanup for user '$PYPI_USER'..."
python3 -m pypi_cleanup.__init__ "${CLEANUP_ARGS[@]}" --do-it --yes -u "$PYPI_USER"

echo "Done."
