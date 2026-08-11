"""
Diagnostics for errors in wrapper interface files.

The interface grammar contains optional and repeated expressions which may
backtrack after a declaration has partially matched.  Pyparsing then reports
the error from the enclosing expression, losing the more useful nested
failure.  This module records those nested failures during a module parse and
turns the best one into a source-aware exception.
"""

from __future__ import annotations

from contextvars import ContextVar, Token
from dataclasses import dataclass
import re
from typing import Optional

from pyparsing import (ParseBaseException, ParserElement, cpp_style_comment,
                       quoted_string)


@dataclass(frozen=True)
class _Failure:
    """A named grammar failure observed during a parse."""

    context: str
    attempted_location: int
    exception: ParseBaseException
    priority: int
    sequence: int

    @property
    def location(self) -> int:
        """Location at which the underlying expression failed."""
        return self.exception.loc

    @property
    def progress(self) -> int:
        """Number of characters consumed before the expression failed."""
        return self.location - self.attempted_location


class _DiagnosticContext:
    """Failure collection state for one module parse."""

    def __init__(self, source: str, source_name: str):
        self.source = source
        self.source_name = source_name
        self.failures: list[_Failure] = []

    def record(self, context: str, attempted_location: int,
               exception: ParseBaseException, priority: int) -> None:
        self.failures.append(
            _Failure(
                context=context,
                attempted_location=attempted_location,
                exception=exception,
                priority=priority,
                sequence=len(self.failures),
            ))

    def best_failure(self, fallback: ParseBaseException) -> _Failure:
        """Select the farthest, most specific credible parser failure."""
        fallback_failure = _Failure(
            context="interface declaration",
            attempted_location=fallback.loc,
            exception=fallback,
            priority=0,
            sequence=len(self.failures),
        )
        return max(
            [*self.failures, fallback_failure],
            key=lambda failure: (
                failure.location,
                failure.priority,
                failure.progress,
                failure.sequence,
            ),
        )


_ACTIVE_CONTEXT: ContextVar[Optional[_DiagnosticContext]] = ContextVar(
    "gtwrap_interface_parser_diagnostic_context", default=None)


class InterfaceParseError(Exception):
    """A source-aware syntax or semantic error in a wrapper interface file."""

    def __init__(
        self,
        *,
        source_name: str,
        source: str,
        location: int,
        context: str,
        expected: str,
        hint: Optional[str] = None,
        cause: Optional[BaseException] = None,
    ):
        self.source_name = source_name
        self.source = source
        self.location = max(0, min(location, len(source)))
        self.context = context
        self.expected = expected
        self.hint = hint
        self.cause = cause

        self.line, self.column, self.line_text = _source_location(
            source, self.location)
        super().__init__(self._summary())

    def _summary(self) -> str:
        return f"parse error in {self.context}: {self.expected}"

    def __str__(self) -> str:
        location = (
            f"{self.source_name}:{self.line}:{self.column}: {self._summary()}")
        if not self.line_text:
            return location

        line_number = str(self.line)
        gutter_width = len(line_number)
        expanded_line = self.line_text.expandtabs(4)
        prefix = self.line_text[:self.column - 1].expandtabs(4)
        caret = " " * len(prefix) + "^"
        rendered = [
            location,
            f" {line_number:>{gutter_width}} | {expanded_line}",
            f" {'':>{gutter_width}} | {caret}",
        ]
        if self.hint:
            rendered.append(f"hint: {self.hint}")
        return "\n".join(rendered)

    @classmethod
    def from_failure(cls, context: _DiagnosticContext,
                     fallback: ParseBaseException) -> "InterfaceParseError":
        """Create an error from the best failure recorded for a parse."""
        failure = context.best_failure(fallback)
        failure_context, expected, hint = _readable_expectation(
            failure.context,
            failure.exception.msg,
            context.source,
            failure.location,
        )
        return cls(
            source_name=context.source_name,
            source=context.source,
            location=failure.location,
            context=failure_context,
            expected=expected,
            hint=hint,
            cause=failure.exception,
        )


def begin_diagnostics(source: str,
                      source_name: str) -> tuple[_DiagnosticContext, Token]:
    """Start collecting parser failures for a module parse."""
    context = _DiagnosticContext(source, source_name)
    return context, _ACTIVE_CONTEXT.set(context)


def end_diagnostics(token: Token) -> None:
    """Restore the diagnostic state active before a module parse."""
    _ACTIVE_CONTEXT.reset(token)


def semantic_error(source: str,
                   location: int,
                   context: str,
                   message: str,
                   hint: Optional[str] = None) -> InterfaceParseError:
    """Create a structured error from a semantic parse-action check."""
    active = _ACTIVE_CONTEXT.get()
    source_name = active.source_name if active else "<string>"
    return InterfaceParseError(
        source_name=source_name,
        source=source,
        location=location,
        context=context,
        expected=message,
        hint=hint,
    )


def track_rule(rule: ParserElement, context: str, priority: int) -> None:
    """Record failures of a meaningful grammar rule while parsing a module."""

    def record_failure(_source: str, location: int, _expression: ParserElement,
                       exception: ParseBaseException) -> None:
        active = _ACTIVE_CONTEXT.get()
        if active is not None:
            active.record(context, location, exception, priority)

    rule.set_fail_action(record_failure)


def _source_location(source: str, location: int) -> tuple[int, int, str]:
    """Return one-based line and column plus the containing source line."""
    line_number = source.count("\n", 0, location) + 1
    line_start = source.rfind("\n", 0, location) + 1
    line_end = source.find("\n", location)
    if line_end == -1:
        line_end = len(source)
    return line_number, location - line_start + 1, source[line_start:line_end]


def _readable_expectation(
        context: str, message: str, source: str,
        location: int) -> tuple[str, str, Optional[str]]:
    """Convert pyparsing's grammar representation into a user-facing error."""
    found = source[location:location + 1]
    previous = source[:location].rstrip()

    if location == len(source):
        unclosed = _last_unclosed_delimiter(source)
        if unclosed:
            opening, closing = unclosed
            return (
                "interface declaration",
                f"expected '{closing}' to close '{opening}'",
                None,
            )

    access_specifier = re.search(r"\b(public|private|protected)\s*$", previous)
    if found == ":" and access_specifier:
        access = access_specifier.group(1)
        return (
            "class member declaration",
            f"access specifier '{access}:' is not supported",
            "list public wrapper declarations without an access label",
        )

    if context == "argument":
        if found in (",", ")"):
            return (
                context,
                "expected argument name",
                "wrapper interface arguments require names, for example "
                "'const Pose3& pose'",
            )
        return context, "expected an argument type followed by its name", None

    if context == "enumerator":
        hint = None
        if found == "}" and previous.endswith(","):
            hint = "remove the trailing comma after the final enumerator"
        return context, "expected enumerator name", hint

    if context == "templated type" and found not in ("", ",", ">"):
        return context, "expected ',' or '>' after template argument", None

    expected = message
    if expected.startswith("Expected "):
        expected = expected[len("Expected "):]
    expected = expected.replace("string_end", "end of file")

    if expected.startswith(("{", "[", "Forward:")) or "W:(" in expected:
        expected = context

    if expected == "';'":
        expected = f"';' after {context}"

    return context, f"expected {expected}", None


def _last_unclosed_delimiter(source: str) -> Optional[tuple[str, str]]:
    """Return the final unmatched delimiter in source, if there is one."""
    closing_for = {"(": ")", "[": "]", "{": "}"}
    opening_for = {closing: opening for opening, closing in closing_for.items()}
    ignored_ranges = iter(
        (start, end) for _, start, end in
        (cpp_style_comment | quoted_string).scan_string(source))
    ignored = next(ignored_ranges, None)
    stack: list[str] = []
    for location, character in enumerate(source):
        while ignored and location >= ignored[1]:
            ignored = next(ignored_ranges, None)
        if ignored and ignored[0] <= location < ignored[1]:
            continue
        if character in closing_for:
            stack.append(character)
        elif character in opening_for:
            if stack and stack[-1] == opening_for[character]:
                stack.pop()
    if not stack:
        return None
    opening = stack[-1]
    return opening, closing_for[opening]
