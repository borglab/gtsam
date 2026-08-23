"""Tests for source-aware wrapper interface parser diagnostics."""

import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from gtwrap.interface_parser import InterfaceParseError, Module


class TestParserDiagnostics(unittest.TestCase):
    """Verify syntax and semantic failures identify the useful source token."""

    def assert_parse_error(self,
                           source,
                           *,
                           line,
                           column,
                           context,
                           expected,
                           source_name="example.i"):
        """Parse source and assert the structured diagnostic fields."""
        with self.assertRaises(InterfaceParseError) as raised:
            Module.parse_string(source, source_name=source_name)

        error = raised.exception
        self.assertEqual(error.source_name, source_name)
        self.assertEqual(error.line, line)
        self.assertEqual(error.column, column)
        self.assertEqual(error.context, context)
        self.assertEqual(error.expected, expected)
        self.assertIn(f"{source_name}:{line}:{column}", str(error))
        self.assertIn("^", str(error))
        return error

    def test_missing_method_semicolon(self):
        self.assert_parse_error(
            "class Foo { void bar() } ;",
            line=1,
            column=24,
            context="method declaration",
            expected="expected ';' after method declaration",
        )

    def test_missing_argument_name(self):
        source = """namespace gtsam {
class Foo {
  void project(const Pose3&);
};
}
"""
        error = self.assert_parse_error(
            source,
            line=3,
            column=28,
            context="argument",
            expected="expected argument name",
        )
        self.assertEqual(error.line_text,
                         "  void project(const Pose3&);")
        self.assertIn("arguments require names", error.hint)
        self.assertIsNotNone(error.cause)

    def test_malformed_template(self):
        self.assert_parse_error(
            "class Foo { void bar(std::vector<double x); };",
            line=1,
            column=41,
            context="templated type",
            expected="expected ',' or '>' after template argument",
        )

    def test_trailing_enum_comma(self):
        error = self.assert_parse_error(
            "enum Color { Red, Green, };",
            line=1,
            column=26,
            context="enumerator",
            expected="expected enumerator name",
        )
        self.assertIn("trailing comma", error.hint)

    def test_unexpected_access_specifier(self):
        error = self.assert_parse_error(
            "class Foo { public: void bar(); };",
            line=1,
            column=19,
            context="class member declaration",
            expected="access specifier 'public:' is not supported",
        )
        self.assertIn("without an access label", error.hint)

    def test_unknown_callable_annotation(self):
        error = self.assert_parse_error(
            "@pybind_adapter int function();",
            line=1,
            column=1,
            context="callable annotation",
            expected="malformed or unknown annotation '@pybind_adapter'",
        )
        self.assertIn("@pybind_lambda", error.hint)

    def test_malformed_callable_annotation(self):
        self.assert_parse_error(
            "@pybind-lambda int function();",
            line=1,
            column=1,
            context="callable annotation",
            expected="malformed or unknown annotation '@pybind-lambda'",
        )

    def test_misplaced_callable_annotation(self):
        error = self.assert_parse_error(
            "class Foo { @pybind_lambda Foo(); };",
            line=1,
            column=13,
            context="callable annotation",
            expected=(
                "annotation '@pybind_lambda' can only be applied to a method, "
                "static method, or global function"
            ),
        )
        self.assertIn("immediately before the callable", error.hint)

    def test_misplaced_annotation_after_template(self):
        self.assert_parse_error(
            "class Foo { template<T> @pybind_lambda Foo(T value); };",
            line=1,
            column=25,
            context="callable annotation",
            expected=(
                "annotation '@pybind_lambda' can only be applied to a method, "
                "static method, or global function"
            ),
        )

    def test_nested_namespace_failure(self):
        self.assert_parse_error(
            "namespace ns { class Foo { void bar(???); }; }",
            line=1,
            column=37,
            context="argument",
            expected="expected an argument type followed by its name",
        )

    def test_unclosed_namespace(self):
        self.assert_parse_error(
            "namespace ns { class Foo { void bar(); }; ",
            line=1,
            column=43,
            context="interface declaration",
            expected="expected '}' to close '{'",
        )

    def test_constructor_name_validation_is_located(self):
        self.assert_parse_error(
            "class Foo { Bar(); };",
            line=1,
            column=13,
            context="constructor declaration",
            expected="constructor name 'Bar' must match class name 'Foo'",
        )

    def test_operator_arity_validation_is_located(self):
        self.assert_parse_error(
            "class Foo { Foo operator*(Foo x, Foo y) const; };",
            line=1,
            column=13,
            context="operator declaration",
            expected="operator overload must have at most one argument; found 2",
        )

    def test_diagnostic_state_does_not_leak(self):
        with self.assertRaises(InterfaceParseError):
            Module.parse_string("class Broken { void method(???); };",
                               source_name="broken.i")

        module = Module.parse_string("class Valid { Valid(); };")
        self.assertEqual(module.content[0].name, "Valid")

    def test_pybind_cli_prints_clean_diagnostic(self):
        repo = Path(__file__).resolve().parents[1]
        script = repo / "scripts" / "pybind_wrap.py"
        template = repo / "tests" / "pybind_wrapper.tpl"

        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "broken.i"
            output = Path(directory) / "broken.cpp"
            source.write_text(
                "class Broken { void method(const Pose3&); };",
                encoding="utf-8",
            )
            environment = os.environ.copy()
            environment["PYTHONPATH"] = str(repo)
            result = subprocess.run(
                [
                    sys.executable,
                    str(script),
                    "--src",
                    str(source),
                    "--module_name",
                    "broken",
                    "--out",
                    str(output),
                    "--template",
                    str(template),
                ],
                cwd=repo,
                env=environment,
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertFalse(output.exists())

        self.assertEqual(result.returncode, 1)
        self.assertIn(f"{source}:1:40: parse error in argument",
                      result.stderr)
        self.assertNotIn("Traceback", result.stderr)

    def test_operator_validation_survives_optimized_python(self):
        repo = Path(__file__).resolve().parents[1]
        environment = os.environ.copy()
        environment["PYTHONPATH"] = str(repo)
        code = """
from gtwrap.interface_parser import InterfaceParseError, Module
try:
    Module.parse_string(
        "class Foo { Foo operator*(Foo x, Foo y) const; };")
except InterfaceParseError:
    raise SystemExit(0)
raise SystemExit(1)
"""
        result = subprocess.run(
            [sys.executable, "-O", "-c", code],
            cwd=repo,
            env=environment,
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
