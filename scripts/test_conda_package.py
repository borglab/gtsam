"""Run the installed Conda package's Python suites with ``python -I``.

The working directory and import checks prevent the checkout or its build tree
from supplying a module that is missing from the package.
"""

from contextlib import chdir
import importlib
import importlib.machinery
from pathlib import Path
import sys
from tempfile import TemporaryDirectory


def main() -> int:
    if not sys.flags.isolated:
        raise RuntimeError("Run this check with python -I")

    prefix = Path(sys.prefix).resolve()
    print(f"Installed-package Python: {sys.version}", flush=True)
    print(f"Installed-package prefix: {prefix}", flush=True)
    with TemporaryDirectory(prefix="gtsam-conda-test-") as directory, chdir(directory):
        for name in ("gtsam", "gtsam_unstable"):
            for module_name in (name, f"{name}.{name}"):
                module = importlib.import_module(module_name)
                location = Path(module.__file__).resolve()
                if not location.is_relative_to(prefix):
                    raise RuntimeError(f"{module_name} loaded outside {prefix}: {location}")
                if module_name != name and not any(
                    str(location).endswith(suffix)
                    for suffix in importlib.machinery.EXTENSION_SUFFIXES
                ):
                    raise RuntimeError(f"{module_name} is not an extension: {location}")
                print(f"{module_name}: {location}", flush=True)

        import pytest

        return pytest.main([
            "-v", "--import-mode=importlib", "--pyargs",
            "gtsam.tests", "gtsam_unstable.tests",
        ])


if __name__ == "__main__":
    sys.exit(main())
