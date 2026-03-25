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

import socket
import time

from zeroconf import DNSOutgoing, DNSQuestion, Zeroconf, const

from raph_fw.console import get_logger, log_step

# ruff: noqa: SLF001

_RAPHCORE_MDNS_NAME = "raphcore.local."


def resolve_raphcore_name(timeout: float = 2.0) -> list[str]:
    """
    Resolve the RaphCore's mDNS name to an IP address using zeroconf.

    :param timeout: Maximum time in seconds to wait for a response.
    :returns: A list of resolved IP addresses.
    :raises TimeoutError: If the name could not be resolved within the timeout.
    """
    zc = Zeroconf()  # Uses all available interfaces by default
    try:
        # 1. Create the outgoing packet (multicast by default)
        out = DNSOutgoing(const._FLAGS_QR_QUERY)

        # 2. Add ONLY the A-record question
        question = DNSQuestion(_RAPHCORE_MDNS_NAME, const._TYPE_A, const._CLASS_IN)
        out.add_question(question)

        # 3. Send the packet
        zc.send(out)

        # 4. Wait the full timeout to aggregate all responses
        time.sleep(timeout)

        # 5. Collect all cached A-record answers
        cached_entries = zc.cache.get_all_by_details(
            _RAPHCORE_MDNS_NAME,
            const._TYPE_A,
            const._CLASS_IN,
        )
        if cached_entries:
            return [socket.inet_ntoa(r.address) for r in cached_entries if hasattr(r, "address")]

        raise TimeoutError
    finally:
        zc.close()


if __name__ == "__main__":
    logger = get_logger("resolve_raphcore_name")

    try:
        with log_step(f"Resolving {_RAPHCORE_MDNS_NAME} to IP address"):
            result = resolve_raphcore_name()
    except TimeoutError:
        logger.warning(f"Failed to resolve {_RAPHCORE_MDNS_NAME} within timeout")
    else:
        logger.info(f"Resolved {_RAPHCORE_MDNS_NAME} to: {result}")
