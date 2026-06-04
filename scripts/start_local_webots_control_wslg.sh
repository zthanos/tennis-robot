#!/usr/bin/env bash
set -euo pipefail

control_port=8081
open_control_panel=1
start_control_panel=1
restart_control_panel=0
stop_existing=1
build_images=0
adapter_name="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-RX 7900 XTX}"

usage() {
  cat <<'EOF'
Usage: bash scripts/start_local_webots_control_wslg.sh [options]

Starts the ROS 2 Webots stack through WSLg, so Docker uses the WSL GPU path
instead of noVNC/Xvfb software rendering.

Options:
  --build                 Rebuild Docker images before starting.
  --port PORT             Control panel port (default: 8081).
  --adapter NAME          Mesa D3D12 adapter name (default: RX 7900 XTX).
  --no-control-panel      Start only Docker/Webots.
  --no-open-control-panel Do not open the control panel URL in Windows.
  --restart-control-panel Stop an existing listener on the control panel port.
  --no-stop-existing      Do not run docker compose down before starting.
  -h, --help              Show this help.

Equivalent intent to:
  .\scripts\start_local_webots_control.ps1 -Ros2Build

but using the ros2-wslg Docker profile.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build)
      build_images=1
      shift
      ;;
    --port)
      control_port="${2:-}"
      if [[ -z "$control_port" ]]; then
        echo "Missing value for --port" >&2
        exit 2
      fi
      shift 2
      ;;
    --adapter)
      adapter_name="${2:-}"
      if [[ -z "$adapter_name" ]]; then
        echo "Missing value for --adapter" >&2
        exit 2
      fi
      shift 2
      ;;
    --no-control-panel)
      start_control_panel=0
      shift
      ;;
    --no-open-control-panel)
      open_control_panel=0
      shift
      ;;
    --restart-control-panel)
      restart_control_panel=1
      shift
      ;;
    --no-stop-existing)
      stop_existing=0
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "$control_port" in
  ''|*[!0-9]*)
    echo "--port must be a number" >&2
    exit 2
    ;;
esac

if [[ ! -e /dev/dxg ]]; then
  echo "ERROR: /dev/dxg is missing. Run this script inside a WSL distro with WSLg/GPU enabled." >&2
  exit 1
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd -- "$script_dir/.." && pwd)"
runtime_dir="$root/runtime"
command_file="$runtime_dir/robot_command.json"
status_file="$runtime_dir/robot_status.json"
sensor_file="$runtime_dir/robot_sensors.json"
control_out="$runtime_dir/control_panel.out.log"
control_err="$runtime_dir/control_panel.err.log"
control_pid_file="$runtime_dir/control_panel.pid"
uv_cache_dir="$runtime_dir/uv-cache"

mkdir -p "$runtime_dir" "$uv_cache_dir"

wait_control_panel() {
  local url="$1"
  local timeout_seconds="${2:-25}"
  local deadline=$((SECONDS + timeout_seconds))

  while (( SECONDS < deadline )); do
    if command -v curl >/dev/null 2>&1; then
      if curl -fsS --max-time 2 "$url" >/dev/null 2>&1; then
        return 0
      fi
    elif command -v python3 >/dev/null 2>&1; then
      if python3 - "$url" >/dev/null 2>&1 <<'PY'
import sys
from urllib.request import urlopen
with urlopen(sys.argv[1], timeout=2) as response:
    raise SystemExit(0 if response.status == 200 else 1)
PY
      then
        return 0
      fi
    fi
    sleep 0.5
  done
  return 1
}

python_has_control_deps() {
  local python_bin="$1"
  "$python_bin" - <<'PY' >/dev/null 2>&1
import importlib.util
raise SystemExit(0 if importlib.util.find_spec("duckdb") else 1)
PY
}

port_pids() {
  local port="$1"
  if command -v ss >/dev/null 2>&1; then
    ss -ltnp "sport = :$port" 2>/dev/null |
      sed -nE 's/.*pid=([0-9]+).*/\1/p' |
      sort -u
  elif command -v lsof >/dev/null 2>&1; then
    lsof -tiTCP:"$port" -sTCP:LISTEN 2>/dev/null | sort -u
  fi
}

stop_control_panel() {
  if [[ -f "$control_pid_file" ]]; then
    while IFS= read -r pid; do
      if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
        echo "Stopping previous control panel pid=$pid..."
        kill "$pid" 2>/dev/null || true
      fi
    done < "$control_pid_file"
    rm -f "$control_pid_file"
  fi

  if (( restart_control_panel )); then
    while IFS= read -r pid; do
      if [[ "$pid" =~ ^[0-9]+$ ]]; then
        echo "Stopping listener on port $control_port pid=$pid..."
        kill "$pid" 2>/dev/null || true
      fi
    done < <(port_pids "$control_port")
  fi
}

start_control_ui() {
  local url="http://127.0.0.1:$control_port"

  if [[ -n "$(port_pids "$control_port")" ]]; then
    echo "Control panel port $control_port already in use; using existing listener."
  else
    local python_cmd=()
    if [[ -n "${CONTROL_PANEL_PYTHON:-}" ]] && command -v "$CONTROL_PANEL_PYTHON" >/dev/null 2>&1; then
      if python_has_control_deps "$CONTROL_PANEL_PYTHON"; then
        python_cmd=("$CONTROL_PANEL_PYTHON")
      else
        echo "WARNING: CONTROL_PANEL_PYTHON is missing control-panel deps such as duckdb." >&2
      fi
    elif command -v uv >/dev/null 2>&1; then
      python_cmd=(uv run python)
    elif command -v python3 >/dev/null 2>&1 && python_has_control_deps python3; then
      python_cmd=(python3)
    elif command -v python >/dev/null 2>&1 && python_has_control_deps python; then
      python_cmd=(python)
    else
      echo "WARNING: Could not find a WSL Python environment with the control-panel deps." >&2
      echo "Docker stack is still running. To enable the control panel, install uv in WSL and rerun:" >&2
      echo "  curl -LsSf https://astral.sh/uv/install.sh | sh" >&2
      echo "  source ~/.bashrc" >&2
      echo "  cd $root" >&2
      echo "  bash scripts/start_local_webots_control_wslg.sh --no-stop-existing" >&2
      return 0
    fi

    echo "Starting control panel on $url ..."
    (
      cd "$root"
      export UV_CACHE_DIR="$uv_cache_dir"
      export ROBOT_COMMAND_FILE="$command_file"
      export ROBOT_STATUS_FILE="$status_file"
      export ROBOT_SENSOR_FILE="$sensor_file"
      exec "${python_cmd[@]}" scripts/control_panel.py \
        --host 127.0.0.1 \
        --port "$control_port" \
        --command-file "$command_file" \
        --status-file "$status_file"
    ) >"$control_out" 2>"$control_err" &
    echo "$!" > "$control_pid_file"
  fi

  echo "Waiting for control UI..."
  if wait_control_panel "$url" 25; then
    echo "Remote control UI is ready: $url"
    if (( open_control_panel )); then
      if command -v powershell.exe >/dev/null 2>&1; then
        powershell.exe -NoProfile -Command "Start-Process '$url'" >/dev/null 2>&1 || true
      elif command -v cmd.exe >/dev/null 2>&1; then
        cmd.exe /c start "" "$url" >/dev/null 2>&1 || true
      fi
    fi
  else
    echo "WARNING: Control UI did not answer yet. Logs:" >&2
    echo "  $control_out" >&2
    echo "  $control_err" >&2
  fi
}

cd "$root"

export MESA_D3D12_DEFAULT_ADAPTER_NAME="$adapter_name"
export ROBOT_COMMAND_FILE="$command_file"
export ROBOT_STATUS_FILE="$status_file"
export ROBOT_SENSOR_FILE="$sensor_file"

if (( start_control_panel )); then
  stop_control_panel
fi

if (( stop_existing )); then
  docker compose --profile ros2-wslg down
fi

compose_args=(--profile ros2-wslg up -d)
if (( build_images )); then
  compose_args+=(--build)
fi
compose_args+=(webots-ros2-wslg ros2-nodes)

echo "Starting ROS 2 WSLg Docker stack (adapter: $MESA_D3D12_DEFAULT_ADAPTER_NAME)..."
docker compose "${compose_args[@]}"

if (( start_control_panel )); then
  start_control_ui
fi

echo
echo "ROS 2 WSLg stack running in Docker"
echo "  Webots:       native WSLg window"
if (( start_control_panel )); then
  echo "  Control UI:   http://127.0.0.1:$control_port"
fi
echo "  Command file: $command_file"
echo
echo "Verify GPU:"
echo "  docker compose --profile ros2-wslg exec webots-ros2-wslg bash -lc 'unset LIBGL_ALWAYS_SOFTWARE; glxinfo -B | grep -E \"Device:|Accelerated|OpenGL renderer\"'"
echo
echo "Useful logs:"
echo "  docker compose --profile ros2-wslg logs -f webots-ros2-wslg"
echo "  docker compose --profile ros2-wslg logs -f ros2-nodes"
