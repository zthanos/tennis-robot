"""Publish ONE discrete event message and confirm it was handed to a subscriber.

Actuator setpoints are idempotent, so RosService can assert them as a short
burst and let repetition cover DDS discovery races. A feed request is not a
setpoint: each message is a separate event. The burst therefore turned one
throw into up to five feed requests, and `ros2 topic pub` — which waits for
only ONE matching subscription before publishing and then tears the publisher
down — still dropped whole throws when more than one node was listening. The
first live E2E run asked for 3 throws, put 10 messages on the wire and got 1
accepted at the consumer.

This publishes exactly once, after waiting for the expected subscriber count,
and lingers briefly so the middleware can flush before the process exits.
Combined with the consumer-side de-duplication on throw_id, delivery is
at-least-once and consumption is idempotent.

Usage:
    python3 -m tennis_robot.reliable_event_publish --topic /throwing/feed_request \\
        --json '{"session_id": "...", "throw_id": "...", "count": 1}'
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from std_msgs.msg import String

DEFAULT_MATCH_TIMEOUT_S = 5.0
SUBSCRIBER_SETTLE_S = 1.0
LINGER_S = 0.3


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Publish one discrete event.")
    parser.add_argument("--topic", required=True)
    parser.add_argument("--json", required=True, help="Payload for a std_msgs/String.")
    parser.add_argument("--min-subscribers", type=int, default=1)
    parser.add_argument("--match-timeout-s", type=float, default=DEFAULT_MATCH_TIMEOUT_S)
    parser.add_argument("--settle-s", type=float, default=SUBSCRIBER_SETTLE_S,
                        help="Keep discovering after the first match, so every "
                             "already-running subscriber receives the event.")
    args = parser.parse_args(argv)

    rclpy.init()
    node = rclpy.create_node("reliable_event_publisher")
    publisher = node.create_publisher(String, args.topic, 10)
    try:
        deadline = time.monotonic() + args.match_timeout_s
        while time.monotonic() < deadline:
            if publisher.get_subscription_count() >= args.min_subscribers:
                break
            rclpy.spin_once(node, timeout_sec=0.05)
        # One match is not "everyone". A short-lived publisher discovers
        # subscribers one at a time, and publishing at the first match delivered
        # each throw to exactly one of two listeners in the live run. Keep
        # spinning after the first match so the remaining already-running
        # subscribers can complete discovery too.
        if publisher.get_subscription_count() >= args.min_subscribers:
            settle = time.monotonic() + args.settle_s
            while time.monotonic() < settle:
                rclpy.spin_once(node, timeout_sec=0.05)
        matched = publisher.get_subscription_count()
        if matched < args.min_subscribers:
            print(
                f"no subscriber matched on {args.topic} within "
                f"{args.match_timeout_s:g}s (saw {matched}); event NOT published",
                file=sys.stderr,
            )
            return 1
        publisher.publish(String(data=args.json))
        # Give the middleware time to flush before the publisher is destroyed.
        linger = time.monotonic() + LINGER_S
        while time.monotonic() < linger:
            rclpy.spin_once(node, timeout_sec=0.05)
        print(f"published 1 event on {args.topic} to {matched} subscriber(s)")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
