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


import logging

from rich.console import Console
from rich.logging import RichHandler
from rich.progress import Progress

console = Console(log_path=False)

logging.basicConfig(
    level=logging.INFO,
    format="%(message)s",
    datefmt="[%X]",
    handlers=[RichHandler(console=console, show_path=False)],
)


def get_logger(name: str = "raph_fw") -> logging.Logger:
    """Get a logger that uses the shared Rich console."""
    return logging.getLogger(name)


def get_progress(**kwargs: dict) -> Progress:
    """Create a Rich Progress instance with the shared console."""
    return Progress(console=console, **kwargs)


if __name__ == "__main__":
    log = get_logger()
    # Logging works normally
    log.info("Starting work...")

    import time

    # Progress bar — pass the same console so logs render above the bar
    with get_progress() as progress:
        task = progress.add_task("Processing...", total=100)
        for i in range(100):
            time.sleep(0.1)
            progress.update(task, advance=1)

    log.info("Done!")
