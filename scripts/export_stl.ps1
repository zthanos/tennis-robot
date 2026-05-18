$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
$outDir = Join-Path $root "cad\3d-printable-base\stl"
New-Item -ItemType Directory -Force $outDir | Out-Null

$models = @(
  "base_tile",
  "motor_pod",
  "drive_wheel_direct_hub",
  "stabilizer_foot",
  "handle_socket"
)

foreach ($model in $models) {
  $source = "cad/3d-printable-base/$model.scad"
  $target = "cad/3d-printable-base/stl/$model.stl"
  Write-Host "Exporting $target"
  docker compose --profile cad run --rm openscad openscad -o $target $source
}
