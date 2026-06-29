from tennis_robot.collector_driver import CollectorDriver, SerialProtocol
from tennis_robot.collector_interface import CollectorInterface


class FakeDriver(CollectorDriver):
    def __init__(self):
        self.values = []

    def set_speed(self, speed_rad_s):
        self.values.append(speed_rad_s)


def test_manual_override_blocks_automatic_commands():
    driver = FakeDriver()
    collector = CollectorInterface(driver)
    collector.start()
    collector.apply_automatic(0.0)
    assert driver.values == [10.0]
    assert collector.status.manual_override


def test_stop_and_speed_adjustment():
    driver = FakeDriver()
    collector = CollectorInterface(driver)
    collector.start()
    collector.adjust_speed(2.0)
    collector.stop()
    assert driver.values == [10.0, 12.0, 0.0]


def test_validated_serial_protocol_bytes():
    assert SerialProtocol.direction(1) == b"f"
    assert SerialProtocol.direction(-1) == b"r"
    assert SerialProtocol.direction(0) == b"s"
    assert SerialProtocol.speed_step(True) == b"+"
    assert SerialProtocol.speed_step(False) == b"-"
