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
  PYPI_CLEANUP_TOTP_SECRET  Optional base32 TOTP seed for PyPI two-factor login

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

run_pypi_cleanup() {
  if [[ -z "${PYPI_CLEANUP_TOTP_SECRET:-}" ]]; then
    python3 -m pypi_cleanup.__init__ "$@"
    return
  fi

  local totp_code
  totp_code=$(python3 - <<'PY'
import base64
import hashlib
import hmac
import os
import struct
import time

secret = os.environ["PYPI_CLEANUP_TOTP_SECRET"].replace(" ", "").upper()
secret += "=" * ((8 - len(secret) % 8) % 8)
key = base64.b32decode(secret, casefold=True)
counter = int(time.time()) // 30
digest = hmac.new(key, struct.pack(">Q", counter), hashlib.sha1).digest()
offset = digest[-1] & 0x0F
code = struct.unpack(">I", digest[offset:offset + 4])[0] & 0x7FFFFFFF
print(f"{code % 1000000:06d}")
PY
)
  printf '%s\n' "$totp_code" | python3 -m pypi_cleanup.__init__ "$@"
}

diagnose_pypi_cleanup_failure() {
  local version="$1"

  python3 - "$PACKAGE" "$PYPI_USER" "$version" <<'PY'
import base64
import hashlib
import hmac
import os
import re
import struct
import sys
import time
from html.parser import HTMLParser
from urllib.parse import urljoin

import requests

package, username, version = sys.argv[1:4]
password = os.environ.get("PYPI_CLEANUP_PASSWORD")
totp_secret = os.environ.get("PYPI_CLEANUP_TOTP_SECRET", "")
base_url = "https://pypi.org"


class FormParser(HTMLParser):
    def __init__(self, action_pattern=None):
        super().__init__()
        self.action_pattern = action_pattern
        self.forms = []
        self._current = None
        self.title = ""
        self._in_title = False

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if tag == "title":
            self._in_title = True
            return
        if tag == "form":
            action = attrs.get("action", "")
            if self.action_pattern is None or re.search(self.action_pattern, action):
                self._current = {"action": action, "inputs": {}}
            return
        if self._current is not None and tag == "input":
            name = attrs.get("name")
            if name:
                self._current["inputs"][name] = attrs.get("value", "")

    def handle_endtag(self, tag):
        if tag == "title":
            self._in_title = False
            return
        if tag == "form" and self._current is not None:
            self.forms.append(self._current)
            self._current = None

    def handle_data(self, data):
        if self._in_title:
            self.title += data


def csrf_for(response_text, action_pattern=None):
    parser = FormParser(action_pattern)
    parser.feed(response_text)
    for form in parser.forms:
        token = form["inputs"].get("csrf_token")
        if token:
            return token, form
    return None, None


def totp_code(secret):
    secret = secret.replace(" ", "").upper()
    secret += "=" * ((8 - len(secret) % 8) % 8)
    key = base64.b32decode(secret, casefold=True)
    counter = int(time.time()) // 30
    digest = hmac.new(key, struct.pack(">Q", counter), hashlib.sha1).digest()
    offset = digest[-1] & 0x0F
    code = struct.unpack(">I", digest[offset:offset + 4])[0] & 0x7FFFFFFF
    return f"{code % 1000000:06d}"


def describe_page(response, reauth_was_required):
    parser = FormParser()
    parser.feed(response.text)
    title = " ".join(parser.title.split()) or "(no title)"
    print(f"PyPI cleanup diagnostic: GET {response.url} returned HTTP {response.status_code}, title {title!r}.")
    if reauth_was_required and "confirm_delete_version" in response.text:
        print("PyPI cleanup diagnostic: PyPI required password reauthentication before exposing the release delete form; pypi-cleanup 0.1.10 does not handle that flow.")
    elif "confirm_delete_version" in response.text:
        print("PyPI cleanup diagnostic: the release delete form is present; pypi-cleanup likely failed to parse PyPI's current HTML.")
    elif "Confirm password to continue" in response.text or "account/reauthenticate" in response.text:
        print("PyPI cleanup diagnostic: PyPI is requesting password reauthentication before showing the release delete form.")
    elif "/account/login/" in response.url:
        print("PyPI cleanup diagnostic: the session was redirected to login, so CI did not stay authenticated.")
    elif "Delete release" not in response.text:
        print("PyPI cleanup diagnostic: PyPI did not render release delete controls. Check that PYPI_CLEANUP_USERNAME is an Owner of the project, not only a Maintainer.")
    else:
        print("PyPI cleanup diagnostic: PyPI rendered the release page but not the expected confirm_delete_version input.")


if not username or not password:
    print("PyPI cleanup diagnostic skipped: username or password is missing.")
    raise SystemExit(0)

session = requests.Session()
session.headers.update({"User-Agent": "gtsam-pypi-cleanup-diagnostic"})

login_url = f"{base_url}/account/login/"
login_page = session.get(login_url, timeout=30)
login_page.raise_for_status()
csrf, _ = csrf_for(login_page.text, r"/account/login/?$")
if not csrf:
    print("PyPI cleanup diagnostic: could not find the login CSRF token.")
    raise SystemExit(0)

login_response = session.post(
    login_url,
    data={"csrf_token": csrf, "username": username, "password": password},
    headers={"referer": login_url},
    timeout=30,
)
login_response.raise_for_status()
if login_response.url.rstrip("/") == login_url.rstrip("/"):
    print("PyPI cleanup diagnostic: PyPI rejected the username or password during login.")
    raise SystemExit(0)

if login_response.url.startswith(f"{base_url}/account/two-factor/"):
    if not totp_secret:
        print("PyPI cleanup diagnostic: PyPI requires two-factor auth, but PYPI_CLEANUP_TOTP_SECRET is not set.")
        raise SystemExit(0)
    csrf, _ = csrf_for(login_response.text, r"/account/two-factor/")
    if not csrf:
        print("PyPI cleanup diagnostic: could not find the two-factor CSRF token.")
        raise SystemExit(0)
    login_response = session.post(
        login_response.url,
        data={"csrf_token": csrf, "method": "totp", "totp_value": totp_code(totp_secret)},
        headers={"referer": login_response.url},
        timeout=30,
    )
    login_response.raise_for_status()
    if login_response.url.startswith(f"{base_url}/account/two-factor/"):
        print("PyPI cleanup diagnostic: PyPI rejected the generated TOTP code.")
        raise SystemExit(0)

release_url = f"{base_url}/manage/project/{package}/release/{version}/"
release_page = session.get(release_url, timeout=30)
release_page.raise_for_status()

reauth_was_required = False
if "Confirm password to continue" in release_page.text or "account/reauthenticate" in release_page.text:
    reauth_was_required = True
    csrf, form = csrf_for(release_page.text, r"/account/reauthenticate/?$")
    if csrf and form:
        reauth_url = urljoin(base_url, form["action"])
        inputs = dict(form["inputs"])
        inputs["csrf_token"] = csrf
        inputs["password"] = password
        reauth_response = session.post(
            reauth_url,
            data=inputs,
            headers={"referer": release_page.url},
            timeout=30,
        )
        reauth_response.raise_for_status()
        release_page = session.get(release_url, timeout=30)
        release_page.raise_for_status()

describe_page(release_page, reauth_was_required)
PY
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
  run_pypi_cleanup "${CLEANUP_ARGS[@]}" --query-only
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
if run_pypi_cleanup "${CLEANUP_ARGS[@]}" --do-it --yes -u "$PYPI_USER"; then
  echo "Done."
else
  cleanup_status=$?
  echo "pypi_cleanup failed; running a PyPI access diagnostic for '${DELETE_VERSIONS[0]}'..." >&2
  diagnose_pypi_cleanup_failure "${DELETE_VERSIONS[0]}" || true
  exit "$cleanup_status"
fi
