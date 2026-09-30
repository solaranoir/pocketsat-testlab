"""EchoTarget behavior beyond the shared contract."""

import pytest

from pocketsat.targets.base import EnvironmentState, TargetFault
from pocketsat.targets.echo import MUTE_FAULT, EchoTarget


@pytest.fixture
def echo() -> EchoTarget:
    target = EchoTarget()
    target.connect()
    target.reset(seed=0)
    return target


def test_echoes_frames_in_order_after_advance(echo: EchoTarget) -> None:
    echo.send(b"a")
    echo.send(b"b")
    assert echo.receive() == []
    echo.advance(0.1)
    assert echo.receive() == [b"a", b"b"]


def test_mute_drops_frames_until_duration_expires(echo: EchoTarget) -> None:
    echo.inject(TargetFault(fault_type=MUTE_FAULT, duration_s=0.2))
    echo.send(b"x")
    echo.advance(0.1)
    assert echo.receive() == []
    echo.send(b"y")
    echo.advance(0.1)
    assert echo.receive() == []
    echo.send(b"z")
    echo.advance(0.1)
    assert echo.receive() == [b"z"]


def test_mute_without_duration_lasts_until_reset(echo: EchoTarget) -> None:
    echo.inject(TargetFault(fault_type=MUTE_FAULT))
    for _ in range(5):
        echo.send(b"x")
        echo.advance(10.0)
        assert echo.receive() == []
    echo.reset(seed=0)
    echo.send(b"x")
    echo.advance(0.1)
    assert echo.receive() == [b"x"]


def test_records_last_environment(echo: EchoTarget) -> None:
    env = EnvironmentState(sunlit=False)
    echo.apply_environment(env)
    assert echo.environment == env
    echo.reset(seed=0)
    assert echo.environment is None


def test_negative_dt_rejected(echo: EchoTarget) -> None:
    with pytest.raises(ValueError, match="non-negative"):
        echo.advance(-0.1)
