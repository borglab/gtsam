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


# Direct includes may target any module reachable through these dependencies.
MODULE_DEPENDENCIES = {
    "base": (),
    "inference": ("base",),
    "geometry": ("inference",),
    "symbolic": ("inference",),
    "discrete": ("inference",),
    "linear": ("symbolic",),
    "nonlinear": ("linear",),
    "hybrid": ("discrete", "geometry", "nonlinear"),
    "basis": ("geometry", "nonlinear"),
    "constrained": ("geometry", "nonlinear"),
    "certifiable": ("constrained",),
    "sam": ("constrained",),
    "slam": ("sam",),
    "sfm": ("slam",),
    "navigation": ("slam",),
    "planning": ("geometry", "nonlinear"),
}

# Existing upward includes. Keep each exception scoped to one file and header.
MODULE_INCLUDE_EXCEPTIONS = {
    # Version-gated 4.3 compatibility paths for the Kruskal header move.
    ("gtsam/base/kruskal-inl.h", "gtsam/inference/kruskal-inl.h"),
    ("gtsam/base/kruskal.h", "gtsam/inference/kruskal.h"),
    ("gtsam/base/numericalDerivative.h", "gtsam/nonlinear/Values.h"),
    ("gtsam/basis/CumulativeSplineTrajectory.h", "gtsam/slam/expressions.h"),
    ("gtsam/discrete/DecisionTreeFactor.cpp", "gtsam/hybrid/HybridValues.h"),
    ("gtsam/discrete/DiscreteConditional.cpp", "gtsam/hybrid/HybridValues.h"),
    ("gtsam/discrete/DiscreteFactor.cpp", "gtsam/hybrid/HybridValues.h"),
    ("gtsam/discrete/TableFactor.cpp", "gtsam/hybrid/HybridValues.h"),
    ("gtsam/geometry/Similarity2.cpp", "gtsam/slam/KarcherMeanFactor-inl.h"),
    ("gtsam/geometry/Similarity3.cpp", "gtsam/slam/KarcherMeanFactor-inl.h"),
    ("gtsam/geometry/triangulation.cpp", "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"),
    ("gtsam/geometry/triangulation.h", "gtsam/nonlinear/NonlinearFactorGraph.h"),
    ("gtsam/geometry/triangulation.h", "gtsam/slam/TriangulationFactor.h"),
    ("gtsam/inference/EliminateableFactorGraph-inst.h", "gtsam/symbolic/IndexedJunctionTree.h"),
    ("gtsam/inference/JunctionTree-inst.h", "gtsam/symbolic/SymbolicConditional.h"),
    ("gtsam/inference/JunctionTree-inst.h", "gtsam/symbolic/SymbolicFactor-inst.h"),
    ("gtsam/inference/graph.h", "gtsam/nonlinear/Values.h"),
    ("gtsam/linear/GaussianConditional.cpp", "gtsam/hybrid/HybridValues.h"),
    ("gtsam/linear/GaussianFactor.cpp", "gtsam/hybrid/HybridValues.h"),
    ("gtsam/nonlinear/NonlinearEquality.h", "gtsam/constrained/NonlinearEquality.h"),
    ("gtsam/nonlinear/NonlinearFactor.cpp", "gtsam/hybrid/HybridValues.h"),
    ("gtsam/nonlinear/NonlinearFactor.h", "gtsam/hybrid/HybridValues.h"),
    ("gtsam/nonlinear/NonlinearMultifrontalSolver.cpp", "gtsam/constrained/NonlinearEqualityConstraint.h"),
    ("gtsam/nonlinear/utilities.h", "gtsam/geometry/Point2.h"),
    ("gtsam/nonlinear/utilities.h", "gtsam/geometry/Point3.h"),
    ("gtsam/nonlinear/utilities.h", "gtsam/geometry/Pose2.h"),
    ("gtsam/nonlinear/utilities.h", "gtsam/geometry/Pose3.h"),
    ("gtsam/nonlinear/utilities.h", "gtsam/geometry/Cal3_S2.h"),
    ("gtsam/nonlinear/utilities.h", "gtsam/geometry/PinholeCamera.h"),
    ("gtsam/nonlinear/utilities.h", "gtsam/slam/ProjectionFactor.h"),
    ("gtsam/slam/EssentialMatrixConstraint.h", "gtsam/sfm/EssentialMatrixConstraint.h"),
    ("gtsam/slam/EssentialMatrixFactor.h", "gtsam/sfm/EssentialMatrixFactor.h"),
}
NON_LIBRARY_DIRS = {"doc", "examples", "tests", "timing", "wrap"}


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


def allowed_dependencies(module):
    """Yield the modules below this module in the dependency graph."""
    for dependency in MODULE_DEPENDENCIES[module]:
        yield dependency
        yield from allowed_dependencies(dependency)


def check_module_include(path, header, module, allowed):
    """Report direct includes outside the library module's dependency graph."""
    target_parts = PurePosixPath(header).parts
    if len(target_parts) < 3 or target_parts[0] != "gtsam":
        return None
    target = target_parts[1]
    if (
        target in MODULE_DEPENDENCIES
        and target != module
        and target not in allowed
        and (path, header) not in MODULE_INCLUDE_EXCEPTIONS
    ):
        return (
            f"Module {module} cannot include {target}. See ARCHITECTURE.md "
            "and place the shared dependency in a lower-level module."
        )
    return None


def check_source(path, source):
    """Yield (line number, diagnostic) for each convention violation."""
    parts = PurePosixPath(path).parts
    stable = path.startswith("gtsam/")
    module = parts[1] if stable and len(parts) >= 3 else None
    check_modules = (
        module in MODULE_DEPENDENCIES
        and not NON_LIBRARY_DIRS.intersection(parts[2:-1])
        and not path.endswith(".i")
    )
    allowed = set(allowed_dependencies(module)) if check_modules else set()
    for token in TOKENS.finditer(source):
        header = token.group("header")
        identifier = token.group("identifier")
        message = None
        if header and stable:
            include = posixpath.normpath(header)
            if not include.startswith(("gtsam/", "gtsam_unstable/")):
                include = posixpath.normpath(posixpath.join(posixpath.dirname(path), header))
            if (
                include.startswith("gtsam_unstable/")
                and (path, include) != UNSTABLE_WRAPPER_INCLUDE
            ):
                message = (
                    "Stable code cannot include gtsam_unstable. Move reusable "
                    "functionality into gtsam or keep the caller experimental."
                )
            elif check_modules:
                message = check_module_include(path, include, module, allowed)
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
