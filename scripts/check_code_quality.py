#!/usr/bin/env python3
"""Check GTSAM-specific source conventions that can be enforced mechanically."""

from pathlib import Path, PurePosixPath
import posixpath
import re
import subprocess


SOURCE_SUFFIXES = {".h", ".hpp", ".c", ".cc", ".cpp", ".cxx", ".cu", ".cuh", ".i"}

# Existing wrapper registration of an experimental filter.
UNSTABLE_WRAPPER_INCLUDE = (
    "gtsam/navigation/navigation_geometric.i",
    "gtsam_unstable/geometry/ABCEquivariantFilter.h",
)

# Declaration and regression test of the 4.3 compatibility alias.
DEPRECATED_EXCEPTION_FILES = {
    "gtsam/linear/linearExceptions.h",
    "gtsam/linear/tests/testLinearExceptions.cpp",
}

# Match direct includes, identifiers, comments and literals in lexical order.
# Ignoring comments and literals avoids reporting examples in documentation.
TOKENS = re.compile(
    r'(?P<include>^[ \t]*#[ \t]*include[ \t]*[<"](?P<header>[^>"\n]+)[>"])'
    r'|//[^\n]*|/\*[\s\S]*?\*/'
    r'|R"(?P<delimiter>[^\s()\\]{0,16})\([\s\S]*?\)(?P=delimiter)"'
    r'|"(?:\\[\s\S]|[^"\\])*"'
    r"|(?<!\w)(?:u8|[uUL])?'(?:\\[^\n]|[^'\\\n])*'"
    r'|(?P<identifier>\b(?:GTSAM_UNSTABLE_EXPORT|GTSAM_EXPORT|'
    r'IndeterminantLinearSystemException)\b)',
    re.MULTILINE,
)


def check_source(path, source):
    """Yield (line number, diagnostic) for each convention violation."""
    stable = path.startswith("gtsam/")
    for token in TOKENS.finditer(source):
        header = token.group("header")
        identifier = token.group("identifier")
        message = None
        if header and stable:
            relative = posixpath.normpath(
                posixpath.join(str(PurePosixPath(path).parent), header)
            )
            if (
                header.startswith("gtsam_unstable/")
                or relative.startswith("gtsam_unstable/")
            ) and (path, header) != UNSTABLE_WRAPPER_INCLUDE:
                message = (
                    "Stable code cannot include gtsam_unstable. Move reusable "
                    "functionality into gtsam or keep the caller experimental."
                )
        elif identifier == "GTSAM_UNSTABLE_EXPORT" and stable:
            message = "Use GTSAM_EXPORT for symbols owned by the stable library."
        elif identifier == "GTSAM_EXPORT" and not stable:
            message = "Use GTSAM_UNSTABLE_EXPORT for symbols owned by the unstable library."
        elif (
            identifier == "IndeterminantLinearSystemException"
            and path not in DEPRECATED_EXCEPTION_FILES
        ):
            message = "Use IndeterminateSystemException (the supported spelling)."

        if message:
            yield source.count("\n", 0, token.start()) + 1, message


def main():
    root = Path(__file__).resolve().parents[1]
    paths = subprocess.check_output(
        ["git", "ls-files", "-z", "--", "gtsam", "gtsam_unstable"], cwd=root
    ).decode().split("\0")

    count = 0
    for path in paths:
        if (
            not path
            or path.startswith("gtsam/3rdparty/")
            or PurePosixPath(path).suffix not in SOURCE_SUFFIXES
            or not (root / path).is_file()
        ):
            continue
        for line, message in check_source(path, (root / path).read_text(encoding="utf-8")):
            print(f"{path}:{line}: {message}")
            count += 1

    if count:
        print(f"Found {count} GTSAM code quality violation(s).")
        return 1
    print("GTSAM code quality checks passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
