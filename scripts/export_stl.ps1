$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
$outDir = Join-Path $root "cad\3d-printable-base\stl"
New-Item -ItemType Directory -Force $outDir | Out-Null

$models = @(
  "base_tile",
  "base_mounting_plate",
  "motor_pod",
  "drive_wheel_direct_hub",
  "stabilizer_foot",
  "handle_socket",
  "collector_breadboard_base",
  "collector_curved_scoop"
)

foreach ($model in $models) {
  $source = "cad/3d-printable-base/$model.scad"
  $target = "cad/3d-printable-base/stl/$model.stl"
  Write-Host "Exporting $target"
  docker compose --profile cad run --rm openscad openscad -o $target $source
}

Write-Host "Exporting detachable collector mounting ear"
docker compose --profile cad run --rm openscad openscad `
  -o cad/3d-printable-base/stl/collector_curved_scoop_mounting_ear.stl `
  cad/3d-printable-base/collector_curved_scoop_mounting_ear.scad

Write-Host "Exporting collector roller"
docker compose --profile cad run --rm openscad openscad `
  -o cad/3d-printable-base/stl/collector_roller.stl `
  cad/3d-printable-base/collector_roller.scad
