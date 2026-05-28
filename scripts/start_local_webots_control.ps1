param(
  [int]$ControlPort = 8081,
  [string]$WebotsPath = "",
  [switch]$NoControlPanel,
  [switch]$NoWebots,
  [switch]$RestartWebots
)

$ErrorActionPreference = "Stop"

function Resolve-WebotsPath {
  param([string]$ExplicitPath)

  if ($ExplicitPath) {
    if (Test-Path $ExplicitPath) {
      return (Resolve-Path $ExplicitPath).Path
    }
    throw "Webots executable not found at: $ExplicitPath"
  }

  if ($env:WEBOTS_HOME) {
    $fromHome = Join-Path $env:WEBOTS_HOME "msys64\mingw64\bin\webots.exe"
    if (Test-Path $fromHome) {
      return (Resolve-Path $fromHome).Path
    }
  }

  $fromPath = Get-Command webots.exe -ErrorAction SilentlyContinue
  if ($fromPath) {
    return $fromPath.Source
  }

  $commonPaths = @(
    "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe",
    "C:\Program Files\Webots\webots.exe"
  )

  foreach ($candidate in $commonPaths) {
    if (Test-Path $candidate) {
      return (Resolve-Path $candidate).Path
    }
  }

  throw "Could not find webots.exe. Set WEBOTS_HOME or pass -WebotsPath 'C:\Path\to\webots.exe'."
}

$root = Split-Path -Parent $PSScriptRoot
$runtimeDir = Join-Path $root "runtime"
$worldFile = Join-Path $root "worlds\tennis_court.wbt"
$commandFile = Join-Path $runtimeDir "robot_command.json"
$statusFile = Join-Path $runtimeDir "robot_status.json"
$sensorFile = Join-Path $runtimeDir "robot_sensors.json"
$controlOut = Join-Path $runtimeDir "control_panel.out.log"
$controlErr = Join-Path $runtimeDir "control_panel.err.log"
$venvPython = Join-Path $root ".venv\Scripts\python.exe"
$webotsPythonCmd = Join-Path $runtimeDir "webots_python.cmd"
$webotsPythonPs1 = Join-Path $runtimeDir "webots_python.ps1"
$pathPythonCmd = Join-Path $runtimeDir "python.cmd"

New-Item -ItemType Directory -Force $runtimeDir | Out-Null

Push-Location $root
try {
  $env:USE_RGB_VISION = "true"
  $env:ROBOT_COMMAND_FILE = $commandFile
  $env:ROBOT_STATUS_FILE = $statusFile
  $env:ROBOT_SENSOR_FILE = $sensorFile

  $uvCommand = Get-Command uv.exe -ErrorAction SilentlyContinue
  if (-not $uvCommand) {
    $uvCommand = Get-Command uv -ErrorAction SilentlyContinue
  }
  if ($uvCommand) {
    $uvPath = $uvCommand.Source
    $psWrapper = @"
param(
  [Parameter(ValueFromRemainingArguments = `$true)]
  [string[]]`$PythonArgs
)

`$ErrorActionPreference = "Stop"
Set-Location "$root"
& "$uvPath" run python @PythonArgs
"@
    Set-Content -Path $webotsPythonPs1 -Value $psWrapper -Encoding ASCII
    $cmdWrapper = @"
@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$webotsPythonPs1" %*
"@
    Set-Content -Path $webotsPythonCmd -Value $cmdWrapper -Encoding ASCII
    $env:WEBOTS_PYTHON_COMMAND = $webotsPythonCmd
  } elseif (Test-Path $venvPython) {
    $env:WEBOTS_PYTHON_COMMAND = $venvPython
  }

  if (Test-Path $venvPython) {
    $pathPythonWrapper = @"
@echo off
cd /d "$root"
"$venvPython" %*
"@
  } elseif ($uvCommand) {
    $uvPath = $uvCommand.Source
    $pathPythonWrapper = @"
@echo off
cd /d "$root"
"$uvPath" run python %*
"@
  } else {
    throw "Could not find project Python or uv. Run 'uv sync' first, or install uv."
  }
  Set-Content -Path $pathPythonCmd -Value $pathPythonWrapper -Encoding ASCII
  $env:PATH = "$runtimeDir;$env:PATH"

  # Write runtime.ini so Webots uses the project Python instead of its bundled msys64 Python.
  # Without this file, Webots ignores WEBOTS_PYTHON_COMMAND and cv2 / other deps are not found.
  $runtimeIniPath = Join-Path $root "controllers\ball_detector\runtime.ini"
  if (Test-Path $venvPython) {
    $pythonForWebots = $venvPython
  } elseif ($uvCommand) {
    $pythonForWebots = & $uvCommand.Source run python -c "import sys; print(sys.executable)" 2>$null
  }
  if ($pythonForWebots -and (Test-Path $pythonForWebots)) {
    Set-Content -Path $runtimeIniPath -Value "[python]`nCOMMAND = $pythonForWebots" -Encoding ASCII
    Write-Host "Webots runtime.ini:  $runtimeIniPath"
    Write-Host "  -> COMMAND = $pythonForWebots"
  } else {
    Write-Warning "Could not resolve project Python for runtime.ini - Webots may use its own Python and miss cv2."
  }

  if (-not $NoWebots) {
    $webotsExe = Resolve-WebotsPath $WebotsPath
    $runningWebots = Get-Process -Name "webots-bin", "webots" -ErrorAction SilentlyContinue
    if ($runningWebots -and $RestartWebots) {
      Write-Host "Stopping existing Webots process so it picks up the project Python environment..."
      $runningWebots | Stop-Process
      Start-Sleep -Seconds 2
    } elseif ($runningWebots) {
      Write-Host "Webots already appears to be running. Close it and rerun this script, or use -RestartWebots, so it picks up the project Python environment."
    }
    Write-Host "Starting local Webots with RGB vision and LiDAR enabled..."
    Write-Host "Webots: $webotsExe"
    Start-Process -FilePath $webotsExe -ArgumentList @($worldFile) -WorkingDirectory $root | Out-Null
  }

  if (-not $NoControlPanel) {
    $existing = Get-NetTCPConnection -LocalPort $ControlPort -State Listen -ErrorAction SilentlyContinue
    if ($existing) {
      Write-Host "Control panel port $ControlPort is already in use; leaving existing listener running."
    } else {
      $controlCommand = @"
`$ErrorActionPreference = 'Stop'
Set-Location '$root'
`$env:USE_RGB_VISION = 'true'
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
  Write-Host "Local Webots world: $worldFile"
  if (-not $NoControlPanel) {
    Write-Host "Remote control UI:  http://127.0.0.1:$ControlPort"
  }
  Write-Host "Vision mode:        USE_RGB_VISION=true"
  Write-Host "Command file:       $commandFile"
  if ($env:WEBOTS_PYTHON_COMMAND) {
    Write-Host "Webots Python:      $env:WEBOTS_PYTHON_COMMAND"
  }
  Write-Host "PATH python shim:   $pathPythonCmd"
  Write-Host ""
  Write-Host "Verify vision after pressing Play in Webots:"
  Write-Host "  Get-Content '$statusFile' | Select-String vision_enabled"
}
finally {
  Pop-Location
}
