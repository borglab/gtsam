"""Temporary native Windows experiment; excluded from the proposed PR."""

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
ROOT = Path.cwd()
OUT = ROOT / "measurements"
OUT.mkdir(exist_ok=True)
PHASE = sys.argv[1]
ENV = os.environ.copy()
if PHASE == "package":
    ENV["SCCACHE_DIR"] = str(ROOT / ".sccache-pkg")
    ENV["SCCACHE_SERVER_PORT"] = "4227"
CACHE = Path(ENV["SCCACHE_DIR"])
SUMMARY = []


def run(args, *, cwd=ROOT, capture=False):
    if PHASE == "package" and args[0] == "sccache":
        args = ["pixi", "exec", "--spec", "sccache", "--", *args]
    print("COMMAND:", args, flush=True)
    process = subprocess.Popen(args, cwd=cwd, env=ENV, text=True,
                               encoding="utf-8", errors="replace",
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    output = []
    with (OUT / "commands.log").open("a", encoding="utf-8") as log:
        log.write(f"COMMAND: {args}\n")
        for line in process.stdout:
            log.write(line)
            if capture:
                output.append(line)
            else:
                print(line, end="", flush=True)
    if process.wait():
        raise subprocess.CalledProcessError(process.returncode, args)
    return "".join(output)


def stats(label):
    value = run(["sccache", "--show-stats", "--stats-format", "json"], capture=True)
    (OUT / f"{label}-stats.json").write_text(value)
    run(["sccache", "--show-stats"])
    return json.loads(value)


def dependencies():
    roots = [ROOT / ".pixi" / "envs" / "test" / "conda-meta"] if PHASE == "development" else [OUT / "package-dependencies"]
    packages = set()
    for root in roots:
        for metadata in root.glob("*.json"):
            record = json.loads(metadata.read_text(encoding="utf-8"))
            packages.add((record["name"], record["version"], record["build"], record.get("sha256", "")))
    if not packages:
        raise RuntimeError("No dependency records found")
    return sorted(packages)


def check_package_rebuild():
    """Prove the warm-build cleanup with a small native package first."""
    project = ROOT / ".benchmark-preflight"
    project.mkdir()
    (project / "pixi.toml").write_text('''[workspace]
name = "gtsam-cache-probe"
channels = ["conda-forge"]
platforms = ["win-64"]
preview = ["pixi-build"]
[package]
name = "gtsam-cache-probe"
version = "0.1.0"
[package.build]
backend = { name = "pixi-build-cmake", version = "0.*" }
[package.build.config]
compilers = ["cxx"]
''')
    (project / "CMakeLists.txt").write_text('''cmake_minimum_required(VERSION 3.24)
project(CacheRebuildProbe LANGUAGES CXX)
add_library(cache_probe SHARED probe.cpp)
install(TARGETS cache_probe)
''')
    (project / "probe.cpp").write_text('__declspec(dllexport) int answer() { return 42; }\n')
    for temperature in ("cold", "warm"):
        if temperature == "warm":
            caches = list((project / ".pixi" / "bld").rglob("CMakeCache.txt"))
            assert caches, "Preflight backend CMake directory not found"
            for cache in caches:
                shutil.rmtree(cache.parent)
        output = run(["pixi", "build", "--path", str(project / "pixi.toml"),
                      "--output-dir", str(project / temperature)], capture=True)
        assert "Building CXX object" in output, "Preflight reused a package without recompiling"
        print(f"RESULT preflight-{temperature}: native compilation observed", flush=True)


def measure(temperature):
    label = f"{PHASE}-{temperature}"
    run(["sccache", "--zero-stats"])
    started = time.monotonic()
    if PHASE == "development":
        run(["pixi", "run", "-e", "test", "build-tests"])
        run(["pixi", "run", "-e", "test", "build-python"])
    else:
        # Cleaning backend artifacts forces recompilation; the compiler cache
        # lives in the checkout, outside this backend build directory.
        run(["pixi", "build", "--path", "pixi.toml", "--build-dir", str(ROOT / "benchmark-package"),
             "--output-dir", f"dist-{temperature}"])
    elapsed = time.monotonic() - started
    current = stats(label)
    size = sum(p.stat().st_size for p in CACHE.rglob("*") if p.is_file())
    records = dependencies()
    (OUT / f"{label}-dependencies.json").write_text(json.dumps(records, indent=2))
    SUMMARY.append({"phase": label, "seconds": elapsed, "cache_bytes": size, "stats": current})
    (OUT / "summary.json").write_text(json.dumps(SUMMARY, indent=2))
    print(f"RESULT {label}: {elapsed:.3f} seconds, {size} cache bytes", flush=True)
    return records


run(["sccache", "--version"])
run([sys.executable, "--version"])
if PHASE == "development":
    run(["cmake", "--version"])
# These jobs intentionally restore no compiler caches.
assert not CACHE.exists(), f"Cold cache already exists: {CACHE}"
# Check the measurement harness before starting the expensive cold build.
run([sys.executable, "-c", "import sys; sys.stdout.reconfigure(encoding='utf-8'); print('UTF-8 log probe: \\U0001f680')"])
stats("empty")
if PHASE == "development":
    dependencies()
else:
    check_package_rebuild()
cold = measure("cold")
if PHASE == "development":
    run(["cmake", "--build", "build", "--target", "clean"])
else:
    # Pixi's frontend build directory is separate from the backend's CMake
    # outputs. Remove those outputs explicitly to force native recompilation.
    caches = list((ROOT / ".pixi" / "bld" / "gtsam").rglob("CMakeCache.txt"))
    assert caches, "No backend CMake build found to clean"
    for cache in caches:
        print("Removing compiler outputs:", cache.parent, flush=True)
        shutil.rmtree(cache.parent)
warm = measure("warm")
assert cold == warm, "Cold and warm dependency records differ"
assert SUMMARY[-1]["stats"]["stats"]["compile_requests"] > 0, "Warm build performed no compiler calls"

if PHASE == "development":
    # Replay an actual formerly PCH-dependent library translation unit. Ninja
    # removes the output both times, so its own up-to-date check cannot hide work.
    target = "gtsam/CMakeFiles/gtsam.dir/base/Vector.cpp.obj"
    for label in ("vector-first", "vector-repeat"):
        run(["sccache", "--zero-stats"])
        (ROOT / "build" / target).unlink()
        run(["ninja", "-C", "build", "-v", target])
        stats(label)

    # MSVC receives native arguments, never MSYS-rewritten /c or /Fo flags.
    probe = OUT / "native-probe"
    probe.mkdir()
    source = probe / "answer.cpp"
    obj = probe / "answer.obj"
    exe = probe / "answer.exe"
    for label, answer in (("source-cold", 41), ("source-repeat", 41), ("source-changed", 42)):
        source.write_text(f'#include <iostream>\nint main() {{ std::cout << {answer}; }}\n')
        obj.unlink(missing_ok=True)
        exe.unlink(missing_ok=True)
        run(["sccache", "--zero-stats"])
        run(["sccache", "cl.exe", "/nologo", "/c", "/O2", "/EHsc", str(source), f"/Fo{obj}"])
        run(["cl.exe", "/nologo", str(obj), f"/Fe{exe}"])
        actual = run([str(exe)], capture=True).strip()
        assert actual == str(answer), (label, actual)
        print(f"RESULT {label}: executable printed {actual}", flush=True)
        stats(label)
    run(["pixi", "run", "-e", "test", "test"])
    run(["pixi", "run", "-e", "test", "python-test"])
    run(["pixi", "run", "-e", "test", "python-test-unstable"])
else:
    artifact = str(next((ROOT / "dist-warm").glob("gtsam-*.conda")).relative_to(ROOT))
    run(["pixi", "exec", "--spec", artifact, "--spec", "pytest", "--", "python", "-I",
         "scripts/test_conda_package.py"])
run(["sccache", "--stop-server"])
