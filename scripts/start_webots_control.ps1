param(
  [int]$ControlPort = 8081,
  [switch]$Build,
  [switch]$NoControlPanel,
  [switch]$NoOpenControlPanel,
  [switch]$RestartControlPanel
)

$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
$runtimeDir = Join-Path $root "runtime"
$commandFile = Join-Path $runtimeDir "robot_command.json"
$statusFile = Join-Path $runtimeDir "robot_status.json"
$sensorFile = Join-Path $runtimeDir "robot_sensors.json"
$controlOut = Join-Path $runtimeDir "control_panel.out.log"
$controlErr = Join-Path $runtimeDir "control_panel.err.log"
$controlLauncher = Join-Path $runtimeDir "start_control_panel.cmd"
$uvCacheDir = Join-Path $runtimeDir "uv-cache"

New-Item -ItemType Directory -Force $runtimeDir | Out-Null
New-Item -ItemType Directory -Force $uvCacheDir | Out-Null

function Test-PythonExecutable {
  param([string]$Path)

  if (-not $Path -or -not (Test-Path $Path)) {
    return $false
  }
  try {
    & $Path --version *> $null
    return $LASTEXITCODE -eq 0
  } catch {
    return $false
  }
}

function Resolve-ControlPanelPython {
  $candidates = @()
  if ($env:CONTROL_PANEL_PYTHON) {
    $candidates += $env:CONTROL_PANEL_PYTHON
  }
  $candidates += (Join-Path $root ".venv\Scripts\python.exe")

  foreach ($commandName in @("python.exe", "python", "py.exe", "py")) {
    $command = Get-Command $commandName -ErrorAction SilentlyContinue
    if ($command) {
      $candidates += $command.Source
    }
  }

  $candidates += @(
    "C:\utils\msys64\mingw64\bin\python.exe",
    "C:\Program Files\LibreOffice\program\python.exe",
    "C:\Program Files\Webots\msys64\mingw64\bin\python.exe"
  )

  foreach ($candidate in $candidates | Select-Object -Unique) {
    if (Test-PythonExecutable $candidate) {
      return $candidate
    }
  }

  return $null
}

function Wait-ControlPanel {
  param(
    [string]$Url,
    [int]$TimeoutSeconds = 20
  )

  $deadline = (Get-Date).AddSeconds($TimeoutSeconds)
  while ((Get-Date) -lt $deadline) {
    try {
      $response = Invoke-WebRequest -UseBasicParsing -Uri $Url -TimeoutSec 2
      if ($response.StatusCode -eq 200) {
        return $true
      }
    } catch {
      Start-Sleep -Milliseconds 500
    }
  }
  return $false
}

Push-Location $root
try {
  $env:USE_RGB_VISION = "true"
  $env:ROBOT_COMMAND_FILE = "/workspace/runtime/robot_command.json"
  $env:ROBOT_STATUS_FILE = "/workspace/runtime/robot_status.json"
  $env:ROBOT_SENSOR_FILE = "/workspace/runtime/robot_sensors.json"
  $env:UV_CACHE_DIR = $uvCacheDir

  if ($Build) {
    Write-Host "Building and starting Webots Docker service with RGB vision and LiDAR enabled..."
    docker compose up -d --build webots
  } else {
    Write-Host "Starting Webots Docker service with RGB vision and LiDAR enabled..."
    docker compose up -d webots
  }

  if (-not $NoControlPanel) {
    $controlUrl = "http://127.0.0.1:$ControlPort"
    $controlPython = Resolve-ControlPanelPython
    $existing = Get-NetTCPConnection -LocalPort $ControlPort -State Listen -ErrorAction SilentlyContinue
    if ($existing -and $RestartControlPanel) {
      Write-Host "Stopping existing control panel listener on port $ControlPort..."
      $existing | Select-Object -ExpandProperty OwningProcess -Unique | ForEach-Object {
        Stop-Process -Id $_ -Force -ErrorAction SilentlyContinue
      }
      Start-Sleep -Seconds 1
      $existing = Get-NetTCPConnection -LocalPort $ControlPort -State Listen -ErrorAction SilentlyContinue
    }

    if ($existing) {
      Write-Host "Control panel port $ControlPort is already in use; using existing listener."
    } else {
      if ($controlPython) {
        $controlPanelCommand = "`"$controlPython`" scripts/control_panel.py --host 127.0.0.1 --port $ControlPort --command-file `"$commandFile`" --status-file `"$statusFile`""
        Write-Host "Control panel Python: $controlPython"
      } else {
        $controlPanelCommand = "uv run python scripts/control_panel.py --host 127.0.0.1 --port $ControlPort --command-file `"$commandFile`" --status-file `"$statusFile`""
        Write-Host "Control panel Python: uv run python"
      }
      $controlLauncherBody = @"
@echo off
cd /d "$root"
set "UV_CACHE_DIR=$uvCacheDir"
set "ROBOT_COMMAND_FILE=$commandFile"
set "ROBOT_STATUS_FILE=$statusFile"
set "ROBOT_SENSOR_FILE=$sensorFile"
$controlPanelCommand > "$controlOut" 2> "$controlErr"
"@
      Set-Content -Path $controlLauncher -Value $controlLauncherBody -Encoding ASCII
      Write-Host "Starting remote control panel on http://127.0.0.1:$ControlPort ..."
      Start-Process -FilePath "cmd.exe" `
        -ArgumentList @("/k", "call `"$controlLauncher`"") `
        -WorkingDirectory $root `
        -WindowStyle Hidden | Out-Null
    }

    Write-Host "Waiting for remote control UI..."
    if (Wait-ControlPanel -Url $controlUrl -TimeoutSeconds 25) {
      Write-Host "Remote control UI is ready: $controlUrl"
      if (-not $NoOpenControlPanel) {
        Start-Process $controlUrl | Out-Null
      }
    } else {
      Write-Warning "Remote control UI did not answer yet. Check logs:"
      Write-Warning "  $controlOut"
      Write-Warning "  $controlErr"
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
