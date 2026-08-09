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

delete_pypi_releases() {
  local candidates_file="$1"
  local cleanup_password="${PYPI_CLEANUP_PASSWORD:-}"

  if [[ -z "$cleanup_password" ]]; then
    [[ -t 0 ]] || die "PYPI_CLEANUP_PASSWORD must be set for noninteractive cleanup"
    read -rsp "Password: " cleanup_password
    echo
  fi

  PYPI_CLEANUP_PASSWORD="$cleanup_password" \
    python3 - "$PACKAGE" "$PYPI_USER" "$candidates_file" 3<&0 <<'PY'
import base64
import hashlib
import hmac
import os
import re
import struct
import sys
import time
from html.parser import HTMLParser
from urllib.parse import urljoin, urlparse

import requests

package, username, candidates_file = sys.argv[1:4]
password = os.environ["PYPI_CLEANUP_PASSWORD"]
totp_secret = os.environ.get("PYPI_CLEANUP_TOTP_SECRET", "")
base_url = "https://pypi.org"

with open(candidates_file, encoding="utf-8") as candidates:
    versions = [line.split("\t", 1)[0] for line in candidates if line.strip()]

if not versions:
    raise SystemExit("No releases were selected for deletion.")

for version in versions:
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._+-]*", version):
        raise SystemExit(f"Refusing unsafe release version {version!r}.")


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


def csrf_for(response_text, action_pattern=None, required_input=None):
    parser = FormParser(action_pattern)
    parser.feed(response_text)
    for form in parser.forms:
        token = form["inputs"].get("csrf_token")
        if token and (required_input is None or required_input in form["inputs"]):
            return token, form
    return None, None


def describe_forms(response):
    parser = FormParser()
    parser.feed(response.text)
    title = " ".join(parser.title.split()) or "(no title)"
    forms = [
        {
            "action": form["action"],
            "fields": sorted(form["inputs"]),
        }
        for form in parser.forms
    ]
    return f"url={response.url!r}, title={title!r}, forms={forms!r}"


def totp_code(secret):
    secret = secret.replace(" ", "").upper()
    secret += "=" * ((8 - len(secret) % 8) % 8)
    key = base64.b32decode(secret, casefold=True)
    counter = int(time.time()) // 30
    digest = hmac.new(key, struct.pack(">Q", counter), hashlib.sha1).digest()
    offset = digest[-1] & 0x0F
    code = struct.unpack(">I", digest[offset:offset + 4])[0] & 0x7FFFFFFF
    return f"{code % 1000000:06d}"


def prompt_for_authentication_code():
    print("Authentication code: ", end="", flush=True)
    with os.fdopen(os.dup(3), encoding="utf-8") as terminal_input:
        authentication_code = terminal_input.readline().strip()
    if not authentication_code:
        raise RuntimeError(
            "PyPI requires two-factor authentication, but no code was supplied."
        )
    return authentication_code


def same_path(url, expected_path):
    return urlparse(url).path.rstrip("/") == expected_path.rstrip("/")


def safe_form_url(action):
    url = urljoin(base_url, action)
    parsed = urlparse(url)
    if parsed.scheme != "https" or parsed.netloc != "pypi.org":
        raise RuntimeError(f"Refusing unexpected PyPI form action {action!r}.")
    return url


session = requests.Session()
session.headers.update({"User-Agent": "gtsam-pypi-cleanup"})

login_url = f"{base_url}/account/login/"
login_page = session.get(login_url, timeout=30)
login_page.raise_for_status()
csrf, _ = csrf_for(login_page.text, r"^/account/login/?(?:\?|$)")
if not csrf:
    raise RuntimeError("Could not find the PyPI login CSRF token.")

login_response = session.post(
    login_url,
    data={"csrf_token": csrf, "username": username, "password": password},
    headers={"referer": login_url},
    timeout=30,
)
login_response.raise_for_status()
if same_path(login_response.url, "/account/login/"):
    raise RuntimeError("PyPI rejected the username or password.")

if login_response.url.startswith(f"{base_url}/account/two-factor/"):
    csrf, _ = csrf_for(login_response.text, r"^/account/two-factor/")
    if not csrf:
        raise RuntimeError("Could not find the PyPI two-factor CSRF token.")
    authentication_code = (
        totp_code(totp_secret)
        if totp_secret
        else prompt_for_authentication_code()
    )
    login_response = session.post(
        login_response.url,
        data={
            "csrf_token": csrf,
            "method": "totp",
            "totp_value": authentication_code,
        },
        headers={"referer": login_response.url},
        timeout=30,
    )
    login_response.raise_for_status()
    if same_path(login_response.url, "/account/confirm-login/"):
        raise RuntimeError(
            "PyPI requires email confirmation for this public IP. Open the "
            "newest unrecognized-login link from the same computer and network, "
            "then rerun this script. GitHub-hosted CI cannot complete this "
            "out-of-band confirmation with password and TOTP secrets alone."
        )
    if same_path(login_response.url, "/account/login/"):
        raise RuntimeError(
            "PyPI returned to the login page after two-factor authentication."
        )
    if login_response.url.startswith(f"{base_url}/account/two-factor/"):
        raise RuntimeError("PyPI rejected the authentication code.")


def release_page_after_reauthentication(version):
    release_path = f"/manage/project/{package}/release/{version}/"
    release_url = f"{base_url}{release_path}"
    release_page = session.get(release_url, timeout=30)
    release_page.raise_for_status()

    if (
        "Confirm password to continue" not in release_page.text
        and "account/reauthenticate" not in release_page.text
    ):
        return release_path, release_url, release_page

    csrf, form = csrf_for(release_page.text, required_input="password")
    if not csrf or not form:
        raise RuntimeError(
            "Could not find the PyPI reauthentication form: "
            + describe_forms(release_page)
        )

    reauth_url = safe_form_url(form["action"])
    if not same_path(reauth_url, "/account/reauthenticate/"):
        raise RuntimeError(
            f"Refusing unexpected reauthentication form action {form['action']!r}."
        )
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
    if same_path(reauth_response.url, "/account/reauthenticate/"):
        raise RuntimeError("PyPI password reauthentication failed.")

    release_page = session.get(release_url, timeout=30)
    release_page.raise_for_status()
    if (
        "Confirm password to continue" in release_page.text
        or "account/reauthenticate" in release_page.text
    ):
        raise RuntimeError(
            "PyPI did not accept password reauthentication: "
            + describe_forms(release_page)
        )
    return release_path, release_url, release_page


delete_requests = []
for version in versions:
    release_path, release_url, release_page = release_page_after_reauthentication(
        version
    )
    csrf, form = csrf_for(
        release_page.text, required_input="confirm_delete_version"
    )
    if not csrf or not form:
        raise RuntimeError(
            f"PyPI did not expose a deletion form for {package!r} {version!r}: "
            + describe_forms(release_page)
        )

    form_url = safe_form_url(form["action"])
    if not same_path(form_url, release_path):
        raise RuntimeError(
            f"Refusing unexpected deletion form action {form['action']!r}."
        )
    delete_requests.append((version, release_path, release_url, form_url, csrf))

print(f"Validated deletion forms for all {len(delete_requests)} release(s).")
print("Deletion starts in 5 seconds; press Ctrl-C now to abort.", flush=True)
time.sleep(5)

for version, release_path, release_url, form_url, csrf in delete_requests:
    print(f"Deleting {package!r} version {version}...", flush=True)
    delete_response = session.post(
        form_url,
        data={
            "csrf_token": csrf,
            "confirm_delete_version": version,
        },
        headers={"referer": release_url},
        timeout=30,
    )
    delete_response.raise_for_status()
    if (
        same_path(delete_response.url, "/account/login/")
        or same_path(delete_response.url, "/account/reauthenticate/")
        or delete_response.url.startswith(f"{base_url}/account/two-factor/")
    ):
        raise RuntimeError(
            f"PyPI requested authentication again while deleting {version!r}."
        )

    verification = session.get(release_url, allow_redirects=False, timeout=30)
    if verification.status_code != 404:
        release_list_path = f"/manage/project/{package}/releases/"
        redirect_url = urljoin(
            base_url, verification.headers.get("location", "")
        )
        if not verification.is_redirect or not same_path(
            redirect_url, release_list_path
        ):
            raise RuntimeError(
                f"PyPI did not confirm removal of {version!r}; "
                "stopping before deleting another release."
            )
    print(f"Deleted {package!r} version {version}.", flush=True)
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

echo "Authenticating with PyPI as '$PYPI_USER'..."
delete_pypi_releases "$CANDIDATES_FILE"
echo "Done."
