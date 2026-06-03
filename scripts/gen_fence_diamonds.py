"""
Replaces the mesh wire sections in all 4 tennis court fences with
diagonal chain-link diamond wires (rhombus pattern).

Usage:  uv run python scripts/gen_fence_diamonds.py [--diamond CM]
Default diamond size: 8 cm  (4 cm is realistic but creates ~4000 nodes)
"""
import re
import sys
import math

DIAMOND_CM = 8  # cm between parallel wire centres
for arg in sys.argv[1:]:
    if arg.startswith("--diamond"):
        DIAMOND_CM = int(arg.split("=")[-1].strip())

D = DIAMOND_CM / 100.0          # metres
WIRE_D = 0.003                   # wire diameter (m)
WIRE_APP = (
    "appearance PBRAppearance { "
    "baseColor 0.13 0.14 0.13 roughness 0.5 metalness 0.7 }"
)


# ── helpers ──────────────────────────────────────────────────────────────────

def _fmt(v: float) -> str:
    """Format float with up to 3 decimal places, no trailing zeros."""
    return f"{v:.3f}".rstrip("0").rstrip(".")


def ew_diagonals(half_height: float = 1.4) -> str:
    """
    East / West fences are thin in X, extend in Y (width) and Z (height).
    '/' wires: rotation 1 0 0 +π/4   (tilts Y-box up-right in YZ plane)
    '\\' wires: rotation 1 0 0 -π/4
    Wire length  = √2 × height  (spans full fence height at 45°)
    Centre range in Y covers the entire 13 m panel width plus half a wire.
    """
    wire_len = round(math.sqrt(2) * 2 * half_height, 4)  # full height diagonal
    box = f"geometry Box {{ size {WIRE_D} {wire_len} {WIRE_D} }}"
    pose_tmpl = (
        "    Pose {{ translation 0 {y} 0 rotation 1 0 0 {r} "
        "children [ Shape {{ {app} {box} }} ] }}"
    )

    # centres must reach from -(6.5+wire_len/2) to +(6.5+wire_len/2)
    half_wire = wire_len / 2
    y_min = -(6.5 + half_wire)
    y_max =  (6.5 + half_wire)

    lines = []
    # '/' set
    y = y_min
    while y <= y_max + 1e-9:
        lines.append(
            pose_tmpl.format(y=_fmt(y), r=" 0.7854", app=WIRE_APP, box=box)
        )
        y = round(y + D, 9)
    # '\' set – offset by D/2 for proper diamond interlock
    y = y_min + D / 2
    while y <= y_max - D / 2 + 1e-9:
        lines.append(
            pose_tmpl.format(y=_fmt(y), r="-0.7854", app=WIRE_APP, box=box)
        )
        y = round(y + D, 9)

    return "\n".join(lines)


def ns_diagonals(half_height: float = 1.4) -> str:
    """
    North / South fences are thin in Y, extend in X (width=26 m) and Z (height).
    '/' wires: rotation 0 1 0 +π/4   (tilts X-box up-right in XZ plane)
    '\\' wires: rotation 0 1 0 -π/4
    """
    wire_len = round(math.sqrt(2) * 2 * half_height, 4)
    box = f"geometry Box {{ size {wire_len} {WIRE_D} {WIRE_D} }}"
    pose_tmpl = (
        "    Pose {{ translation {x} 0 0 rotation 0 1 0 {r} "
        "children [ Shape {{ {app} {box} }} ] }}"
    )

    half_wire = wire_len / 2
    x_min = -(13.0 + half_wire)
    x_max =  (13.0 + half_wire)

    lines = []
    x = x_min
    while x <= x_max + 1e-9:
        lines.append(
            pose_tmpl.format(x=_fmt(x), r=" 0.7854", app=WIRE_APP, box=box)
        )
        x = round(x + D, 9)
    x = x_min + D / 2
    while x <= x_max - D / 2 + 1e-9:
        lines.append(
            pose_tmpl.format(x=_fmt(x), r="-0.7854", app=WIRE_APP, box=box)
        )
        x = round(x + D, 9)

    return "\n".join(lines)


# ── regex-based replacement ───────────────────────────────────────────────────

def replace_fence_wires(content: str, fence_name: str, diag_block: str) -> str:
    """
    Inside the named fence Solid, remove every existing horizontal-wire Pose
    (Box with two equal outer dims and one very small dim) and vertical-wire
    Pose, replacing the whole wire section with diag_block.

    Strategy: locate the fence by its name field, then find and replace the
    contiguous block of Pose lines that contain thin-wire Box geometries.
    """
    # Locate the fence block by name
    name_pat = re.compile(
        r'(name\s+"' + re.escape(fence_name) + r'")',
        re.DOTALL,
    )
    m = name_pat.search(content)
    if not m:
        print(f"  WARNING: fence '{fence_name}' not found")
        return content

    name_pos = m.start()

    # Scan backwards from name_pos to find the children [ ... ] block
    # We want to remove lines that look like wire Poses inside that block.
    # Find the matching opening 'children [' before name_pos.

    # Simplest approach: replace all thin-wire Pose lines (identified by the
    # very small Box dimension pattern: size < 0.02 in any dimension).
    # We operate only within the fence Solid – from 'DEF <FENCE_NAME>' to
    # 'name "<fence_name>"'.

    def_pat = re.compile(
        r'DEF\s+' + re.escape(fence_name.upper()) + r'\s+Solid\s*\{'
    )
    dm = def_pat.search(content)
    if not dm:
        print(f"  WARNING: DEF for '{fence_name}' not found")
        return content

    fence_start = dm.start()
    fence_end   = m.end()          # just past the name line
    fence_block = content[fence_start:fence_end]

    # Remove all existing thin-wire Poses inside the fence block.
    # A thin-wire Pose has Box sizes like: 0.015 13 0.009  or  0.015 0.009 2.8
    # (one very small dim < 0.02, one large dim > 1.0)
    wire_pose_pat = re.compile(
        r'\n\s+Pose\s*\{[^}]*?Box\s*\{[^}]*?\}[^}]*?\}\s*\]\s*\}',
        re.DOTALL
    )

    def is_wire_pose(match_text: str) -> bool:
        box_m = re.search(r'size\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)', match_text)
        if not box_m:
            return False
        dims = [float(box_m.group(i)) for i in (1, 2, 3)]
        # A wire Pose has at least one dim < 0.02 AND at least one dim > 0.5
        return min(dims) < 0.02 and max(dims) > 0.5

    # Remove wire Poses from fence_block
    def remove_wires(text: str) -> str:
        result = []
        i = 0
        for m2 in wire_pose_pat.finditer(text):
            if is_wire_pose(m2.group()):
                result.append(text[i:m2.start()])
                i = m2.end()
        result.append(text[i:])
        return "".join(result)

    cleaned_fence = remove_wires(fence_block)

    # Insert the diagonal block just before the closing '  ]' of children
    # The closing line is '  ]' followed by newline then '  name'
    insert_pat = re.compile(r'(\n\s+\]\n\s+name\s+"' + re.escape(fence_name) + r'")')
    cleaned_fence = insert_pat.sub(
        "\n" + diag_block + r"\1",
        cleaned_fence,
        count=1,
    )

    return content[:fence_start] + cleaned_fence + content[fence_end:]


# ── main ─────────────────────────────────────────────────────────────────────

WBT = "worlds/tennis_court.wbt"

with open(WBT, "r", encoding="utf-8") as f:
    src = f.read()

ew_block = ew_diagonals()
ns_block = ns_diagonals()

ew_count  = ew_block.count("Pose")
ns_count  = ns_block.count("Pose")
print(f"Diamond size : {DIAMOND_CM} cm")
print(f"EW fence     : {ew_count} diagonal wire nodes each")
print(f"NS fence     : {ns_count} diagonal wire nodes each")

for fence in ("east_fence", "west_fence"):
    print(f"  patching {fence}…")
    src = replace_fence_wires(src, fence, ew_block)

for fence in ("north_fence", "south_fence"):
    print(f"  patching {fence}…")
    src = replace_fence_wires(src, fence, ns_block)

with open(WBT, "w", encoding="utf-8") as f:
    f.write(src)

print("Done — worlds/tennis_court.wbt updated.")
