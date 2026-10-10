"""Temporary native Windows experiment; excluded from the proposed PR."""

import json
import os
from pathlib import Path
import subprocess
import sys
import time

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
            record = json.loads(metadata.read_text())
            packages.add((record["name"], record["version"], record["build"], record.get("sha256", "")))
    if not packages:
        raise RuntimeError("No dependency records found")
    return sorted(packages)


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
             "--clean", "--output-dir", f"dist-{temperature}"])
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
run(["python", "--version"])
run(["cmake", "--version"])
# These jobs intentionally restore no compiler caches.
assert not CACHE.exists(), f"Cold cache already exists: {CACHE}"
cold = measure("cold")
if PHASE == "development":
    run(["cmake", "--build", "build", "--target", "clean"])
warm = measure("warm")
assert cold == warm, "Cold and warm dependency records differ"

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
