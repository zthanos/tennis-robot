"""
Remove old grid (horizontal + vertical) wire Pose lines from all 4 fence Solids.
These are single-line Poses with Box sizes that contain both 0.015 and 0.009
as wire-gauge dimensions. New diagonal wires use 0.003 and are NOT removed.
"""
import re

WBT = "worlds/tennis_court.wbt"

# Old grid wire sizes have exactly {0.015, 0.009} as the two thin dims.
# Match any single-line Pose that contains a Box whose size field contains
# both 0.015 and 0.009 (in any order among the 3 dims).
GRID_LINE_PAT = re.compile(
    r'^\s*Pose\s*\{[^\n]*Box\s*\{\s*size\s+[^\}]+\}[^\n]*\}\s*$',
    re.MULTILINE,
)


def is_old_grid_wire(line: str) -> bool:
    m = re.search(r'size\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)', line)
    if not m:
        return False
    dims = {float(m.group(i)) for i in (1, 2, 3)}
    # Old grid wires have both 0.015 and 0.009 as dimensions
    return 0.015 in dims and 0.009 in dims


with open(WBT, "r", encoding="utf-8") as f:
    src = f.read()

removed = 0
result_lines = []
for line in src.splitlines(keepends=True):
    if GRID_LINE_PAT.match(line) and is_old_grid_wire(line):
        removed += 1
    else:
        result_lines.append(line)

new_src = "".join(result_lines)

with open(WBT, "w", encoding="utf-8") as f:
    f.write(new_src)

print(f"Removed {removed} old grid-wire Pose lines.")
print("Done — worlds/tennis_court.wbt updated.")
