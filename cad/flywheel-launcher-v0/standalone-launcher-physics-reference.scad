// Authoritative standalone launcher pose for the isolated physics bench.
// This wrapper adds no geometry and does not import compact-robot packaging.
use <launcher-envelope.scad>

launcher_oriented(
    orientation="side_by_side",
    nip_height=215,
    launch_pitch_deg=20);
