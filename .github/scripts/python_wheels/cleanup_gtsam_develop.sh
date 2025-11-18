#!/usr/bin/env bash

# This script deletes all but the most recent release from the gtsam-develop project on PyPI
# and can be used if the project size exceeds the PyPI limit of 10 GB. The user must have
# owner or maintainer privileges on the project.

set -euo pipefail

usage() {
  cat <<EOF
Usage: $(basename "$0") <pypi_username>

Deletes all but the most recent release from the gtsam-develop project on PyPI.
You must supply the PyPI user name that has owner or maintainer privileges on
the project. THIS OPERATION IS PERMANENT.

Examples
  $ $(basename "$0") yambati3
  $ $(basename "$0")        # will prompt for user name
EOF
}

if [[ $# -ge 1 ]]; then
  PYPI_USER="$1"
else
  read -rp "Enter your PyPI user name: " PYPI_USER
  [[ -z "$PYPI_USER" ]] && { echo "No user name supplied."; usage; exit 1; }
fi

echo "-----------------------------------------------------------------------"
echo "WARNING: This WILL permanently delete all but the most recent release"
echo "         of 'gtsam-develop' on PyPI for user '$PYPI_USER'."
echo "         This cannot be undone."
echo "-----------------------------------------------------------------------"
read -rp "Proceed? [y/N]: " REPLY
REPLY=$(echo "$REPLY" | tr '[:upper:]' '[:lower:]')   # to lowercase
[[ "$REPLY" != "y" && "$REPLY" != "yes" ]] && { echo "Aborted."; exit 0; }

echo "Running pypi_cleanup for user '$PYPI_USER'..."
python3 -m pypi_cleanup.__init__ \
        -p gtsam-develop \
        --leave-most-recent-only \
        --do-it \
        -u "$PYPI_USER"

echo "Done."
