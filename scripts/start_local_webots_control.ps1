param(
  [int]$ControlPort = 8081,
  [string]$WebotsPath = "",
  [switch]$NoControlPanel,
  [switch]$NoOpenControlPanel,
  [switch]$RestartControlPanel,
  [switch]$NoWebots,
  [switch]$RestartWebots,
  [switch]$NoStopExisting,
  # ROS 2 mode: runs the full stack (Webots + behavior nodes) inside Docker
  # instead of local Webots. The control panel still runs locally; the shared
  # runtime/ volume keeps it connected to the Docker stack.
  # Webots noVNC is at http://localhost:6082/vnc.html when this flag is set.
  [switch]$Ros2,
  # Rebuild Docker images before starting (implies -Ros2).
  [switch]$Ros2Build
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
  param(
    [string]$VenvPython,
    [object]$UvCommand
  )

  $candidates = @()
  if ($env:CONTROL_PANEL_PYTHON) {
    $candidates += $env:CONTROL_PANEL_PYTHON
  }
  $candidates += $VenvPython

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

  if ($UvCommand) {
    return $null
  }
  throw "Could not find a working Python for the control panel. Set CONTROL_PANEL_PYTHON to python.exe."
}

$root = Split-Path -Parent $PSScriptRoot
$runtimeDir = Join-Path $root "runtime"
$worldFile = Join-Path $root "worlds\tennis_court.wbt"
$commandFile = Join-Path $runtimeDir "robot_command.json"
$statusFile = Join-Path $runtimeDir "robot_status.json"
$sensorFile = Join-Path $runtimeDir "robot_sensors.json"
$controlOut = Join-Path $runtimeDir "control_panel.out.log"
$controlErr = Join-Path $runtimeDir "control_panel.err.log"
$controlLauncher = Join-Path $runtimeDir "start_control_panel.cmd"
$controlPidFile = Join-Path $runtimeDir "control_panel.pid"
$webotsPidFile = Join-Path $runtimeDir "webots.pid"
$uvCacheDir = Join-Path $runtimeDir "uv-cache"
$venvPython = Join-Path $root ".venv\Scripts\python.exe"
$webotsPythonCmd = Join-Path $runtimeDir "webots_python.cmd"
$webotsPythonPs1 = Join-Path $runtimeDir "webots_python.ps1"
$pathPythonCmd = Join-Path $runtimeDir "python.cmd"

New-Item -ItemType Directory -Force $runtimeDir | Out-Null
New-Item -ItemType Directory -Force $uvCacheDir | Out-Null

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

function Stop-ProcessId {
  param(
    [int]$ProcessId,
    [string]$Label
  )

  $process = Get-Process -Id $ProcessId -ErrorAction SilentlyContinue
  if (-not $process) {
    return
  }
  Write-Host "Stopping previous $Label pid=$ProcessId ($($process.ProcessName))..."
  Stop-Process -Id $ProcessId -Force -ErrorAction SilentlyContinue
}

function Stop-PidFile {
  param(
    [string]$Path,
    [string]$Label
  )

  if (-not (Test-Path $Path)) {
    return
  }

  $ids = Get-Content $Path -ErrorAction SilentlyContinue |
    ForEach-Object {
      $value = 0
      if ([int]::TryParse($_, [ref]$value)) { $value }
    } |
    Select-Object -Unique

  foreach ($id in $ids) {
    Stop-ProcessId -ProcessId $id -Label $Label
  }

  Remove-Item -LiteralPath $Path -Force -ErrorAction SilentlyContinue
}

function Stop-ProcessesByCommandLine {
  param(
    [string]$Label,
    [string[]]$ProcessNames,
    [string[]]$Patterns
  )

  $escapedNames = $ProcessNames | ForEach-Object { "'$($_.Replace("'", "''"))'" }
  $filter = "Name = " + ($escapedNames -join " OR Name = ")
  $processes = Get-CimInstance Win32_Process -Filter $filter -ErrorAction SilentlyContinue
  foreach ($process in $processes) {
    $commandLine = [string]$process.CommandLine
    foreach ($pattern in $Patterns) {
      if ($commandLine -like "*$pattern*") {
        Stop-ProcessId -ProcessId ([int]$process.ProcessId) -Label $Label
        break
      }
    }
  }
}

function Stop-PreviousLocalRun {
  if (-not $NoControlPanel) {
    Stop-PidFile -Path $controlPidFile -Label "control panel launcher"

    $listener = Get-NetTCPConnection -LocalPort $ControlPort -State Listen -ErrorAction SilentlyContinue
    foreach ($ownerProcessId in ($listener | Select-Object -ExpandProperty OwningProcess -Unique)) {
      Stop-ProcessId -ProcessId ([int]$ownerProcessId) -Label "control panel listener on port $ControlPort"
    }

    Stop-ProcessesByCommandLine `
      -Label "control panel command" `
      -ProcessNames @("cmd.exe", "python.exe", "pythonw.exe", "uv.exe", "powershell.exe") `
      -Patterns @($controlLauncher, "scripts/control_panel.py", "scripts\control_panel.py")
  }

  if (-not $NoWebots) {
    Stop-PidFile -Path $webotsPidFile -Label "Webots launcher"
    Stop-ProcessesByCommandLine `
      -Label "Webots world process" `
      -ProcessNames @("webots.exe", "webots-bin.exe") `
      -Patterns @($worldFile, "worlds\tennis_court.wbt", "worlds/tennis_court.wbt")
  }
}

Push-Location $root
try {
  # ── ROS 2 Docker mode ────────────────────────────────────────────────────────
  # Runs the complete ROS 2 stack in Docker: Webots (webots_bridge controller)
  # + behavior nodes. The local control panel still runs on Windows and talks
  # to the Docker stack via the shared runtime/ volume mount.
  if ($Ros2 -or $Ros2Build) {
    if (-not $NoStopExisting) {
      Stop-PreviousLocalRun
    }

    $env:ROBOT_COMMAND_FILE = $commandFile
    $env:ROBOT_STATUS_FILE  = $statusFile
    $env:ROBOT_SENSOR_FILE  = $sensorFile

    $dockerArgs = @("compose", "--profile", "ros2", "up", "-d")
    if ($Ros2Build) { $dockerArgs += "--build" }
    $dockerArgs += @("webots-ros2", "ros2-nodes")

    Write-Host "Starting ROS 2 Docker stack (webots-ros2 + ros2-nodes)..."
    & docker @dockerArgs
    if ($LASTEXITCODE -ne 0) {
      throw "docker compose failed (exit $LASTEXITCODE). Run 'docker compose --profile ros2 logs' to diagnose."
    }

    if (-not $NoControlPanel) {
      $uvCommand = Get-Command uv.exe -ErrorAction SilentlyContinue
      if (-not $uvCommand) { $uvCommand = Get-Command uv -ErrorAction SilentlyContinue }
      $controlPython = Resolve-ControlPanelPython -VenvPython $venvPython -UvCommand $uvCommand
      $controlUrl = "http://127.0.0.1:$ControlPort"

      $existing = Get-NetTCPConnection -LocalPort $ControlPort -State Listen -ErrorAction SilentlyContinue
      if ($existing -and $RestartControlPanel) {
        $existing | Select-Object -ExpandProperty OwningProcess -Unique | ForEach-Object {
          Stop-Process -Id $_ -Force -ErrorAction SilentlyContinue
        }
        Start-Sleep -Seconds 1
        $existing = Get-NetTCPConnection -LocalPort $ControlPort -State Listen -ErrorAction SilentlyContinue
      }

      if ($existing) {
        Write-Host "Control panel port $ControlPort already in use; using existing listener."
      } else {
        if ($controlPython) {
          $controlPanelCommand = "`"$controlPython`" scripts/control_panel.py --host 127.0.0.1 --port $ControlPort --command-file `"$commandFile`" --status-file `"$statusFile`""
        } else {
          $controlPanelCommand = "uv run python scripts/control_panel.py --host 127.0.0.1 --port $ControlPort --command-file `"$commandFile`" --status-file `"$statusFile`""
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
        Write-Host "Starting control panel on $controlUrl ..."
        $controlProcess = Start-Process -FilePath "cmd.exe" `
          -ArgumentList @("/k", "call `"$controlLauncher`"") `
          -WorkingDirectory $root -WindowStyle Hidden -PassThru
        Set-Content -Path $controlPidFile -Value $controlProcess.Id -Encoding ASCII
      }

      Write-Host "Waiting for control UI..."
      if (Wait-ControlPanel -Url $controlUrl -TimeoutSeconds 25) {
        Write-Host "Remote control UI is ready: $controlUrl"
        if (-not $NoOpenControlPanel) { Start-Process $controlUrl | Out-Null }
      } else {
        Write-Warning "Control UI did not answer yet. Logs: $controlOut / $controlErr"
      }
    }

    Write-Host ""
    Write-Host "ROS 2 stack running in Docker"
    Write-Host "  Webots noVNC:  http://localhost:6082/vnc.html"
    if (-not $NoControlPanel) {
      Write-Host "  Control UI:    http://127.0.0.1:$ControlPort"
    }
    Write-Host "  Command file:  $commandFile"
    Write-Host ""
    Write-Host "Useful commands:"
    Write-Host "  docker compose --profile ros2 logs -f webots-ros2"
    Write-Host "  docker compose --profile ros2 logs -f ros2-nodes"
    Write-Host "  docker compose --profile ros2 down"
    Write-Host ""
    Write-Host "NOTE: world controller must be 'webots_bridge' in worlds/tennis_court.wbt line 4653."
    return
  }

  # ── Legacy local-Webots path (default) ──────────────────────────────────────
  if (-not $NoStopExisting) {
    Stop-PreviousLocalRun
  }

  $env:USE_RGB_VISION = "true"
  $env:ROBOT_COMMAND_FILE = $commandFile
  $env:ROBOT_STATUS_FILE = $statusFile
  $env:ROBOT_SENSOR_FILE = $sensorFile
  $env:UV_CACHE_DIR = $uvCacheDir

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
    $webotsProcess = Start-Process -FilePath $webotsExe -ArgumentList @($worldFile) -WorkingDirectory $root -PassThru
    Set-Content -Path $webotsPidFile -Value $webotsProcess.Id -Encoding ASCII
  }

  if (-not $NoControlPanel) {
    $controlUrl = "http://127.0.0.1:$ControlPort"
    $controlPython = Resolve-ControlPanelPython -VenvPython $venvPython -UvCommand $uvCommand
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
set "USE_RGB_VISION=true"
set "ROBOT_COMMAND_FILE=$commandFile"
set "ROBOT_STATUS_FILE=$statusFile"
set "ROBOT_SENSOR_FILE=$sensorFile"
$controlPanelCommand > "$controlOut" 2> "$controlErr"
"@
      Set-Content -Path $controlLauncher -Value $controlLauncherBody -Encoding ASCII
      Write-Host "Starting remote control panel on http://127.0.0.1:$ControlPort ..."
      $controlProcess = Start-Process -FilePath "cmd.exe" `
        -ArgumentList @("/k", "call `"$controlLauncher`"") `
        -WorkingDirectory $root `
        -WindowStyle Hidden `
        -PassThru
      Set-Content -Path $controlPidFile -Value $controlProcess.Id -Encoding ASCII
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
  Write-Host ""
  Write-Host "To run the ROS 2 Docker stack instead:"
  Write-Host "  .\scripts\start_local_webots_control.ps1 -Ros2         # requires controller 'webots_bridge'"
  Write-Host "  .\scripts\start_local_webots_control.ps1 -Ros2Build    # same + rebuild images"
}
finally {
  Pop-Location
}
