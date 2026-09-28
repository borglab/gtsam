"""Check that the version in pixi.toml matches the one in CMakeLists.txt.

pixi cannot derive a package version from CMake yet
(https://github.com/prefix-dev/pixi/issues/2946), so the version is duplicated
in pixi.toml. This fails the build if the two drift apart.
"""

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent


def cmake_version() -> str:
    text = (ROOT / "CMakeLists.txt").read_text()
    parts = []
    for component in ("MAJOR", "MINOR", "PATCH"):
        match = re.search(rf"set\s*\(\s*GTSAM_VERSION_{component}\s+(\d+)\s*\)", text)
        if match is None:
            sys.exit(f"could not find GTSAM_VERSION_{component} in CMakeLists.txt")
        parts.append(match.group(1))
    return ".".join(parts)


def pixi_version() -> str:
    text = (ROOT / "pixi.toml").read_text()
    match = re.search(r'^\s*version\s*=\s*"([^"]+)"', text, re.MULTILINE)
    if match is None:
        sys.exit("could not find a version in pixi.toml")
    return match.group(1)


def main() -> None:
    cmake, pixi = cmake_version(), pixi_version()
    if cmake != pixi:
        sys.exit(
            f"version mismatch: CMakeLists.txt says {cmake}, pixi.toml says {pixi}.\n"
            f"Update the version in pixi.toml to {cmake}."
        )
    print(f"pixi.toml and CMakeLists.txt agree on version {cmake}")


if __name__ == "__main__":
    main()
