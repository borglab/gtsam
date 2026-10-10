"""Temporary native layout diagnosis using the original Windows artifact."""
import importlib
import os
from pathlib import Path
import runpy
import shutil
import sys
import sysconfig
from tempfile import TemporaryDirectory

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
helper = Path(__file__).with_name("test_conda_package.py").resolve()
prefix = Path(sys.prefix)
original = prefix / "Library" / "lib" / "python" / "site-packages"
destination = Path(sysconfig.get_path("purelib"))
with TemporaryDirectory() as working:
    previous = Path.cwd()
    os.chdir(working)
    try:
        try:
            importlib.import_module("gtsam")
        except ModuleNotFoundError as error:
            assert error.name == "gtsam", error
            print("Original import failure:", repr(error), flush=True)
        else:
            raise AssertionError("Original artifact unexpectedly imports")
        assert original.is_dir(), original
        destination.mkdir(parents=True, exist_ok=True)
        for item in original.iterdir():
            print("Relocating:", item, "to", destination, flush=True)
            shutil.move(str(item), destination / item.name)
        importlib.invalidate_caches()
        runpy.run_path(str(helper), run_name="__main__")
    finally:
        os.chdir(previous)
