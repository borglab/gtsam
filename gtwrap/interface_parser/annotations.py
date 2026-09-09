"""Annotations supported by wrapper interface files."""

from pyparsing import Regex

from .diagnostics import semantic_error
from .template import Template


PYBIND_LAMBDA = Regex(r"@pybind_lambda(?![A-Za-z0-9_])")


def _reject_annotation(source, location, tokens):
    """Raise a useful error for unknown or misplaced annotations."""
    annotation = tokens[0]
    context = "callable annotation"
    if annotation == "@pybind_lambda":
        message = (
            "annotation '@pybind_lambda' can only be applied to a method, "
            "static method, or global function"
        )
        hint = (
            "place '@pybind_lambda' after any template declaration and "
            "immediately before the callable declaration"
        )
    elif annotation == "@serializable":
        context = "typedef annotation"
        message = (
            "annotation '@serializable' can only be applied to a template "
            "typedef"
        )
        hint = "place '@serializable' immediately before the typedef declaration"
    else:
        message = f"malformed or unknown annotation '{annotation}'"
        hint = (
            "use a supported annotation such as '@pybind_lambda' or "
            "'@serializable' in its documented position"
        )

    raise semantic_error(
        source,
        location,
        context,
        message,
        hint,
    )


UNSUPPORTED_ANNOTATION = Regex(r"@[^\s;{}()]+|@").set_parse_action(
    _reject_annotation)
UNSUPPORTED_TEMPLATED_ANNOTATION = (
    Template.rule.suppress() + UNSUPPORTED_ANNOTATION)
