#!/usr/bin/env python3
"""Generate tennis ball Solid nodes for the Webots world."""

from __future__ import annotations

import argparse
import random
from dataclasses import dataclass


COURT_LENGTH_M = 23.77
COURT_WIDTH_M = 10.97
BALL_RADIUS_M = 0.033


@dataclass(frozen=True)
class Ball:
    index: int
    x: float
    y: float


def generate_balls(count: int, seed: int, margin: float) -> list[Ball]:
    rng = random.Random(seed)
    half_length = COURT_LENGTH_M / 2 - margin
    half_width = COURT_WIDTH_M / 2 - margin

    balls: list[Ball] = []
    for index in range(count):
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
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    for ball in generate_balls(args.count, args.seed, args.margin):
        print(format_ball(ball))
        print()


if __name__ == "__main__":
    main()
