# Copyright 2026 Fictionlab sp. z o.o.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from __future__ import annotations

import logging
import time
from contextlib import contextmanager
from datetime import UTC, datetime
from typing import TYPE_CHECKING

from rich.console import Console
from rich.live import Live
from rich.logging import RichHandler
from rich.progress import (
    BarColumn,
    Progress,
    ProgressColumn,
    TaskProgressColumn,
    TextColumn,
    TimeRemainingColumn,
)
from rich.prompt import Confirm, IntPrompt
from rich.text import Text

if TYPE_CHECKING:
    from collections.abc import Callable, Generator
    from typing import ParamSpec, TypeVar

    P = ParamSpec("P")
    R = TypeVar("R")

console = Console(log_path=False, log_time=True)

logging.basicConfig(
    level=logging.INFO,
    format="%(message)s",
    datefmt="[%X]",
    handlers=[
        RichHandler(
            console=console,
            show_path=False,
            show_time=True,
            show_level=True,
            markup=True,
        ),
    ],
)


def get_logger(name: str = "raph_fw") -> logging.Logger:
    """
    Get a logger that uses the shared Rich console.

    :param name: The logger name.
    """
    return logging.getLogger(name)


_log = get_logger()


class _LogTimeColumn(ProgressColumn):
    """Render current time in the same style as RichHandler."""

    def render(self, _task: object) -> Text:
        return Text(datetime.now(tz=UTC).astimezone().strftime("[%X]"), style="log.time")


class _LogLevelColumn(ProgressColumn):
    """Render a fixed level label in the same style as RichHandler."""

    def render(self, _task: object) -> Text:
        return Text("INFO".ljust(8), style="logging.level.info")


def _get_log_prefix() -> Text:
    """Get the standard log message prefix (timestamp and level) as Text."""
    return Text.assemble(
        (datetime.now(tz=UTC).astimezone().strftime("[%X]"), "log.time"),
        " ",
        ("INFO    ", "logging.level.info"),
        " ",
    )


def get_progress() -> Progress:
    """Create a Rich Progress instance with the shared console."""
    return Progress(
        _LogTimeColumn(),
        _LogLevelColumn(),
        TextColumn("[progress.description]{task.description}"),
        BarColumn(),
        TaskProgressColumn(),
        TimeRemainingColumn(),
        console=console,
    )


@contextmanager
def log_step(
    label: str,
    success_text: str = "OK",
    failure_text: str = "FAILED",
) -> Generator[None, None, None]:
    """
    Render a CMake-style step line and emit a log record once finished.

    :param label: The description text shown while the step is in progress.
    :param success_text: Text shown when the step completes successfully.
    :param failure_text: Text shown when the step raises an exception.
    """
    start = time.perf_counter()

    live = Live(Text.assemble(_get_log_prefix(), f"{label}..."), console=console, transient=True)
    live.start()

    try:
        yield
    except Exception:
        live.stop()
        elapsed_ms = (time.perf_counter() - start) * 1000
        _log.info(f"{label}... [red]{failure_text}[/red] ({elapsed_ms:.0f} ms)")
        raise
    else:
        live.stop()
        elapsed_ms = (time.perf_counter() - start) * 1000
        _log.info(f"{label}... [green]{success_text}[/green] ({elapsed_ms:.0f} ms)")


def run_step(
    label: str,
    fn: Callable[P, R],
    /,
    *args: P.args,
    **kwargs: P.kwargs,
) -> R:
    """
    Run a callable within log_step and return its result.

    :param label: The description text shown while the step is in progress.
    :param fn: The callable to execute.
    :param args: Positional arguments forwarded to *fn*.
    :param kwargs: Keyword arguments forwarded to *fn*. The special keys
        ``success_text`` and ``failure_text`` are extracted and passed
        to :func:`log_step` instead.
    """
    success_text = str(kwargs.pop("success_text", "OK"))
    failure_text = str(kwargs.pop("failure_text", "FAILED"))
    with log_step(label, success_text=success_text, failure_text=failure_text):
        return fn(*args, **kwargs)


def get_confirmation_prompt(prompt: str, *, default: bool = False) -> bool:
    """
    Display a confirmation prompt styled consistently with logging output.

    :param prompt: The question to display.
    :param default: The default answer when the user presses Enter.
    """
    return Confirm.ask(Text.assemble(_get_log_prefix(), prompt), default=default, console=console)


def get_choice_prompt(prompt: str, choices: list[str], *, default: str | None = None) -> str:
    """
    Display a numbered multiple-choice prompt styled consistently with logging output.

    :param prompt: The question to display above the choices.
    :param choices: The list of options the user can pick from.
    :param default: The default choice if the user presses Enter, or ``None``
        for no default.
    :returns: The selected choice string.
    """
    prefix = _get_log_prefix()
    console.print(Text.assemble(prefix, prompt), end="\n")
    default_idx: int | None = None
    for i, choice in enumerate(choices, 1):
        console.print(" " * len(prefix), f"  {i}. {choice}", end="\n")
        if choice == default:
            default_idx = i
    index = IntPrompt.ask(
        Text.assemble(" " * len(prefix), "Choice"),
        default=default_idx,
        console=console,
    )
    while index is None or index < 1 or index > len(choices):
        index = IntPrompt.ask(
            Text.assemble(_get_log_prefix(), f"Please enter 1-{len(choices)}"),
            console=console,
        )
    return choices[index - 1]


if __name__ == "__main__":
    import contextlib

    log = get_logger()
    log.info("Starting console helper demo")

    with log_step("Checking firmware version"):
        time.sleep(1.0)

    def _connect_to_device(port: str) -> str:
        time.sleep(1.0)
        return f"Connected on {port}"

    connection = run_step("Connecting to device", _connect_to_device, "/dev/ttyACM0")
    log.info(connection)

    with (
        contextlib.suppress(RuntimeError),
        log_step("Running self-test", failure_text="Self-test failed"),
    ):
        time.sleep(1.0)
        raise RuntimeError

    # Progress bar - pass the same console so logs render above the bar.
    with get_progress() as progress:
        task = progress.add_task("Processing...", total=100)
        for _ in range(100):
            time.sleep(0.01)
            progress.update(task, advance=1, refresh=True)

    if get_confirmation_prompt("Continue with demo?", default=True):
        log.info("User confirmed")
    else:
        log.info("User declined")

    selected = get_choice_prompt(
        "Select a device:",
        ["RaphCore v1", "RaphCore v2", "RaphCore v3"],
        default="RaphCore v2",
    )
    log.info(f"Selected: {selected}")

    log.info("Demo complete")
