param(
  [int]$ControlPort = 8081,
  [switch]$Build,
  [switch]$NoControlPanel
)

$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
$runtimeDir = Join-Path $root "runtime"
$commandFile = Join-Path $runtimeDir "robot_command.json"
$statusFile = Join-Path $runtimeDir "robot_status.json"
$sensorFile = Join-Path $runtimeDir "robot_sensors.json"
$controlOut = Join-Path $runtimeDir "control_panel.out.log"
$controlErr = Join-Path $runtimeDir "control_panel.err.log"

New-Item -ItemType Directory -Force $runtimeDir | Out-Null

Push-Location $root
try {
  $env:USE_RGB_VISION = "true"
  $env:ROBOT_COMMAND_FILE = "/workspace/runtime/robot_command.json"
  $env:ROBOT_STATUS_FILE = "/workspace/runtime/robot_status.json"
  $env:ROBOT_SENSOR_FILE = "/workspace/runtime/robot_sensors.json"

  if ($Build) {
    Write-Host "Building and starting Webots Docker service with RGB vision and LiDAR enabled..."
    docker compose up -d --build webots
  } else {
    Write-Host "Starting Webots Docker service with RGB vision and LiDAR enabled..."
    docker compose up -d webots
  }

  if (-not $NoControlPanel) {
    $existing = Get-NetTCPConnection -LocalPort $ControlPort -State Listen -ErrorAction SilentlyContinue
    if ($existing) {
      Write-Host "Control panel port $ControlPort is already in use; leaving existing listener running."
    } else {
      $controlCommand = @"
`$ErrorActionPreference = 'Stop'
Set-Location '$root'
`$env:ROBOT_COMMAND_FILE = '$commandFile'
`$env:ROBOT_STATUS_FILE = '$statusFile'
`$env:ROBOT_SENSOR_FILE = '$sensorFile'
uv run python scripts/control_panel.py --host 127.0.0.1 --port $ControlPort --command-file '$commandFile' --status-file '$statusFile'
"@
      Write-Host "Starting remote control panel on http://127.0.0.1:$ControlPort ..."
      Start-Process powershell.exe `
        -ArgumentList @("-NoExit", "-ExecutionPolicy", "Bypass", "-Command", $controlCommand) `
        -WorkingDirectory $root `
        -RedirectStandardOutput $controlOut `
        -RedirectStandardError $controlErr | Out-Null
    }
  }

  Write-Host ""
  Write-Host "Webots noVNC:       http://localhost:6080/vnc.html"
  if (-not $NoControlPanel) {
    Write-Host "Remote control UI:  http://127.0.0.1:$ControlPort"
  }
  Write-Host "Vision mode:        USE_RGB_VISION=true"
  Write-Host "Command file:       $commandFile"
  Write-Host ""
  Write-Host "Useful logs:"
  Write-Host "  docker compose logs -f webots"
  Write-Host "  Get-Content '$statusFile' | Select-String vision_enabled"
}
finally {
  Pop-Location
}
