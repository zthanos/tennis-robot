from __future__ import annotations

import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _text(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_field_wifi_shell_scripts_parse():
    for path in (
        "scripts/network/install_field_wifi_ap.sh",
        "scripts/network/field_wifi_status.sh",
        "scripts/network/validate_field_wifi.sh",
        "scripts/install_pi_console_service.sh",
    ):
        subprocess.run(["bash", "-n", str(ROOT / path)], check=True)


def test_installer_is_persistent_secure_and_idempotent():
    script = _text("scripts/network/install_field_wifi_ap.sh")
    assert "connection.autoconnect yes" in script
    assert "connection.autoconnect-priority 999" in script
    assert "802-11-wireless.mode ap" in script
    assert "ipv4.method shared" in script
    assert "ipv4.never-default yes" in script
    assert "802-11-wireless.ap-isolation no" in script
    assert "802-11-wireless-security.key-mgmt" in script
    assert "802-11-wireless-security.proto rsn" in script
    assert "802-11-wireless-security.pairwise ccmp" in script
    assert "802-11-wireless-security.group ccmp" in script
    assert 'PMF="disable"' in script
    assert 'PMF="required"' in script
    assert '802-11-wireless-security.pmf "$PMF"' in script
    assert "802-11-wireless-security.wps-method disabled" in script
    assert "connection show | grep -Fxq" in script
    assert "SSH_CONNECTION" in script
    assert "WIFI-PROPERTIES.AP" in script
    assert "dnsmasq" in script
    assert "connection delete" not in script


def test_example_does_not_contain_a_real_secret():
    example = _text("config/network/field-wifi.env.example")
    assert (
        'FIELD_WIFI_PASSPHRASE="REPLACE_WITH_A_UNIQUE_RANDOM_PASSPHRASE"'
        in example
    )
    assert 'FIELD_WIFI_ADDRESS="10.42.0.1/24"' in example


def test_pi_setup_integrates_opt_in_field_wifi_provisioning():
    setup = _text("scripts/setup_pi.sh")
    assert 'INSTALL_FIELD_WIFI:-false' in setup
    assert "install_field_wifi_ap.sh" in setup


def test_operator_console_is_reachable_but_ros_is_not_newly_exposed():
    launch = _text("ros2_ws/src/tennis_robot/launch/sim.launch.py")
    assert '"--host",\n            "0.0.0.0"' in launch
    run_pi = _text("run_pi.sh")
    assert "field_wifi_status" not in run_pi
    assert "phone" not in run_pi.lower()


def test_console_systemd_service_is_non_root_and_network_reachable():
    service = _text("config/systemd/tennis-robot-console.service.in")
    installer = _text("scripts/install_pi_console_service.sh")
    assert "User=@USER@" in service
    assert "Group=@GROUP@" in service
    assert "--host 0.0.0.0 --port 8081" in service
    assert "Restart=on-failure" in service
    assert "WantedBy=multi-user.target" in service
    assert "systemctl enable --now tennis-robot-console.service" in installer
