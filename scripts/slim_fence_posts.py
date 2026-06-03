"""
Slim fence posts and remove the center post from each fence.
  Before: E/W posts 0.12×0.1×2.85 every 2 m (7 posts incl. center)
          N/S posts 0.1×0.12×2.85 every 2 m (13 posts incl. center)
  After:  all posts 0.05×0.05×2.85, center post removed.
"""
import re

WBT = "worlds/tennis_court.wbt"

with open(WBT, "r", encoding="utf-8") as f:
    src = f.read()

# ── 1. Remove the center post lines (translation 0 0 0, height 2.85) ─────────
# Match single-line Poses with translation 0 0 0 and a vertical post Box size.
CENTER_POST = re.compile(
    r'^\s*Pose\s*\{\s*translation\s+0\s+0\s+0\s+children\s*\[.*?'
    r'size\s+(?:0\.12\s+0\.1|0\.1\s+0\.12)\s+2\.85.*?\]\s*\}\s*\n',
    re.MULTILINE,
)

before = len(src)
src = CENTER_POST.sub("", src)
removed = before - len(src)
print(f"Removed center-post lines: {removed} chars")

# ── 2. Slim remaining posts: 0.12 0.1 → 0.05 0.05, 0.1 0.12 → 0.05 0.05 ────
src, n1 = re.subn(r'size 0\.12 0\.1 2\.85', 'size 0.05 0.05 2.85', src)
src, n2 = re.subn(r'size 0\.1 0\.12 2\.85', 'size 0.05 0.05 2.85', src)
print(f"Slimmed E/W posts: {n1}  |  N/S posts: {n2}")

with open(WBT, "w", encoding="utf-8") as f:
    f.write(src)

print("Done — worlds/tennis_court.wbt updated.")
