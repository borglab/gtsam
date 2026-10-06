#!/usr/bin/env python3
"""Exercise compiler-object reuse across deleted builds and restored snapshots."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


SCRIPT = Path(__file__).with_name("compiler_cache.sh").resolve()


def run(*args, env, cwd=None):
    result = subprocess.run(args, env=env, cwd=cwd, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if result.returncode:
        raise AssertionError(f"{args}:\n{result.stdout}")
    return result.stdout


class CompilerCacheTest(unittest.TestCase):
    def exercise(self, tool):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            env = dict(os.environ, RUNNER_TEMP=str(root), RUNNER_OS="Linux",
                       RUNNER_ARCH="X64", CACHE_TOOL=tool, CACHE_LANE="test-gcc",
                       CACHE_BUILD_TYPE="Release", CACHE_CONFIGURATION="fixture",
                       GITHUB_ENV=str(root / "env"), GITHUB_STEP_SUMMARY=str(root / "summary"))
            # Pin both frontends so an inherited CI CC/CXX cannot change the test.
            env.update(CC="gcc", CXX="g++", SCCACHE_SERVER_PORT="4428")
            run("bash", str(SCRIPT), "setup", env=env)
            for line in (root / "env").read_text().splitlines():
                key, value = line.split("=", 1)
                env[key] = value
            cache = Path(env["COMPILER_CACHE_DIR"])
            source = root / "source"
            source.mkdir()
            (source / "CMakeLists.txt").write_text('''cmake_minimum_required(VERSION 3.16)
project(CacheFixture LANGUAGES C CXX)
enable_testing()
set(EXPECTED 42 CACHE STRING "Expected source value")
add_executable(changed main.cpp answer.cpp)
add_executable(unchanged unchanged.c)
add_test(NAME changed COMMAND changed ${EXPECTED})
add_test(NAME unchanged COMMAND unchanged)
''')
            (source / "main.cpp").write_text('''#include <cstdlib>
int answer();
int main(int argc, char** argv) { return argc == 2 && answer() == std::atoi(argv[1]) ? 0 : 1; }
''')
            (source / "answer.cpp").write_text("int answer() { return 42; }\n")
            (source / "unchanged.c").write_text("int main(void) { return 0; }\n")
            build = root / "build"
            try:
                for phase, expected in (("cold", 42), ("warm", 42), ("changed-source", 43)):
                    if build.exists():
                        shutil.rmtree(build)
                    if phase == "changed-source":
                        (source / "answer.cpp").write_text("int answer() { return 43; }\n")
                    # Configure before zeroing statistics to exclude CMake probes.
                    run("cmake", "-S", str(source), "-B", str(build), "-G", "Ninja",
                        "-DCMAKE_BUILD_TYPE=Release", f"-DEXPECTED={expected}",
                        f"-DCMAKE_C_COMPILER_LAUNCHER={tool}",
                        f"-DCMAKE_CXX_COMPILER_LAUNCHER={tool}", env=env)
                    run("bash", str(SCRIPT), "reset", env=env)
                    run("cmake", "--build", str(build), "--parallel", "2", env=env)
                    # CTest runs on every pass, including fully cached compilations.
                    tests = run("ctest", "--test-dir", str(build), "--output-on-failure", env=env)
                    self.assertIn("100% tests passed", tests)
                    if tool == "ccache":
                        stats = dict(line.split() for line in run(tool, "--print-stats", env=env).splitlines())
                        hits = int(stats["direct_cache_hit"]) + int(stats["preprocessed_cache_hit"])
                        misses = int(stats["cache_miss"])
                        self.assertEqual(int(stats["compile_failed"]), 0)
                    else:
                        stats = json.loads(run(tool, "--show-stats", "--stats-format=json", env=env))["stats"]
                        hits = sum(stats["cache_hits"]["counts"].values())
                        misses = sum(stats["cache_misses"]["counts"].values())
                        self.assertEqual(stats["compile_fails"], 0)
                        self.assertEqual(stats["cache_write_errors"], 0)
                        self.assertEqual(sum(stats["cache_errors"]["counts"].values()), 0)
                    self.assertEqual((hits, misses), {"cold": (0, 3), "warm": (3, 0), "changed-source": (2, 1)}[phase])
                    print(f"{tool} {phase}: {hits} hits, {misses} misses, 2/2 tests passed", flush=True)
                    run("bash", str(SCRIPT), "finish", env=env)
                    # Round-trip only compiler objects, never the build tree.
                    archive = shutil.make_archive(str(root / "snapshot"), "gztar", cache)
                    shutil.rmtree(cache)
                    cache.mkdir()
                    shutil.unpack_archive(archive, cache, filter="data")
            finally:
                if tool == "sccache":
                    subprocess.run([tool, "--stop-server"], env=env, capture_output=True)

    def test_namespace(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            env = dict(os.environ, RUNNER_TEMP=str(root), RUNNER_OS="Linux",
                       RUNNER_ARCH="X64", CACHE_TOOL="ccache", CACHE_LANE="gcc-14",
                       CACHE_BUILD_TYPE="Release", CACHE_CONFIGURATION="a")
            def namespace(**changes):
                current = dict(env, **changes, GITHUB_ENV=str(root / "env"))
                (root / "env").write_text("")
                run("bash", str(SCRIPT), "setup", env=current)
                values = dict(line.split("=", 1) for line in (root / "env").read_text().splitlines())
                return values["COMPILER_CACHE_PREFIX"]
            original = namespace()
            self.assertEqual(original, namespace(GITHUB_SHA="changed-source-revision"))
            for changes in ({"CACHE_BUILD_TYPE": "Debug"}, {"CACHE_LANE": "system-libs"},
                            {"CACHE_CONFIGURATION": "b"}, {"RUNNER_ARCH": "ARM64"},
                            {"CXXFLAGS": "-fno-exceptions"}, {"CACHE_TOOL": "sccache"}):
                self.assertNotEqual(original, namespace(**changes))

    def test_ccache(self):
        self.exercise("ccache")

    def test_sccache(self):
        self.exercise("sccache")


if __name__ == "__main__":
    unittest.main()
