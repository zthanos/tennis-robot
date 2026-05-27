#!/usr/bin/env python3
"""Generate tennis ball Solid nodes for the Webots world."""

from __future__ import annotations

import argparse
import random
from dataclasses import dataclass


COURT_LENGTH_M = 23.77
COURT_WIDTH_M = 10.97
BALL_RADIUS_M = 0.033
SERVICE_LINE_X_M = 6.40


@dataclass(frozen=True)
class Ball:
    index: int
    x: float
    y: float


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def sample_realistic_position(rng: random.Random, margin: float) -> tuple[float, float]:
    """Bias balls toward the net and back court, with a few mid-court outliers."""

    half_length = COURT_LENGTH_M / 2 - margin
    half_width = COURT_WIDTH_M / 2 - margin
    side = -1 if rng.random() < 0.5 else 1
    net_x = 0.35 * side
    back_x = half_length * side
    zone = rng.random()

    if zone < 0.46:
        x = net_x + side * abs(rng.gauss(0, 1.05))
        y = rng.gauss(0, COURT_WIDTH_M * 0.26)
    elif zone < 0.82:
        x = back_x - side * abs(rng.gauss(0, 1.20))
        y = rng.gauss(0, COURT_WIDTH_M * 0.30)
    elif zone < 0.94:
        x = SERVICE_LINE_X_M * side + rng.gauss(0, 1.65)
        y = rng.gauss(0, COURT_WIDTH_M * 0.24)
    else:
        x = rng.uniform(-half_length, half_length)
        y = rng.uniform(-half_width, half_width)

    return (
        clamp(x, -half_length, half_length),
        clamp(y, -half_width, half_width),
    )


def generate_balls(count: int, seed: int, margin: float, distribution: str) -> list[Ball]:
    rng = random.Random(seed)
    half_length = COURT_LENGTH_M / 2 - margin
    half_width = COURT_WIDTH_M / 2 - margin

    balls: list[Ball] = []
    for index in range(count):
        if distribution == "realistic":
            x, y = sample_realistic_position(rng, margin)
        else:
            x = rng.uniform(-half_length, half_length)
            y = rng.uniform(-half_width, half_width)
        balls.append(Ball(index=index, x=x, y=y))
    return balls


def format_ball(ball: Ball) -> str:
    return f"""DEF TENNIS_BALL_{ball.index:02d} Solid {{
  translation {ball.x:.3f} {ball.y:.3f} {BALL_RADIUS_M:.3f}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.73 0.95 0.11
        roughness 0.65
        metalness 0
      }}
      geometry Sphere {{
        radius {BALL_RADIUS_M:.3f}
        subdivision 3
      }}
    }}
  ]
  name "tennis_ball_{ball.index:02d}"
  contactMaterial "tennis_ball"
  boundingObject Sphere {{
    radius {BALL_RADIUS_M:.3f}
  }}
  physics Physics {{
    density -1
    mass 0.058
  }}
}}"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--count", type=int, default=18)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--margin", type=float, default=0.7)
    parser.add_argument("--distribution", choices=("realistic", "uniform"), default="realistic")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    for ball in generate_balls(args.count, args.seed, args.margin, args.distribution):
        print(format_ball(ball))
        print()


if __name__ == "__main__":
    main()
