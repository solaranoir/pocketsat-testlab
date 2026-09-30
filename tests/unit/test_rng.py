"""Unit tests for pocketsat.core.rng."""

import hashlib
from random import Random

import pytest

from pocketsat.core.rng import RngFactory, derive_seed


def _draws(rng: Random, n: int = 20) -> list[float]:
    return [rng.random() for _ in range(n)]


def test_derive_seed_matches_adr_0003_definition() -> None:
    expected = int.from_bytes(hashlib.sha256(b"42:rf.loss").digest()[:8], "big")
    assert derive_seed(42, "rf.loss") == expected


def test_derive_seed_known_value() -> None:
    # Pinned so a change to the derivation is caught, not silently accepted.
    assert derive_seed(42, "run:0") == 0xB835EAC0A9329F86


def test_derive_seed_range_and_labels() -> None:
    seeds = {derive_seed(7, f"run:{i}") for i in range(1000)}
    assert len(seeds) == 1000
    assert all(0 <= s < 2**64 for s in seeds)


def test_derive_seed_rejects_non_int() -> None:
    with pytest.raises(TypeError):
        derive_seed(True, "x")
    with pytest.raises(TypeError):
        derive_seed(1.0, "x")  # type: ignore[arg-type]


def test_stream_is_reproducible() -> None:
    assert _draws(RngFactory(42).stream("rf.loss")) == _draws(RngFactory(42).stream("rf.loss"))
    factory = RngFactory(42)
    assert _draws(factory.stream("rf.loss")) == _draws(factory.stream("rf.loss"))


def test_stream_seeded_with_derived_seed() -> None:
    assert _draws(RngFactory(42).stream("env.sensor_noise")) == _draws(
        Random(derive_seed(42, "env.sensor_noise"))
    )


def test_different_names_and_seeds_are_independent() -> None:
    factory = RngFactory(42)
    assert _draws(factory.stream("rf.loss")) != _draws(factory.stream("rf.jitter"))
    assert _draws(RngFactory(42).stream("rf.loss")) != _draws(RngFactory(43).stream("rf.loss"))


def test_new_stream_does_not_change_existing_stream_values() -> None:
    baseline = RngFactory(42)
    loss = baseline.stream("rf.loss")
    expected = [loss.random() for _ in range(10)] + [loss.uniform(0, 5) for _ in range(10)]

    grown = RngFactory(42)
    grown.stream("added.first").random()  # new consumer requested before
    loss = grown.stream("rf.loss")
    first = [loss.random() for _ in range(10)]
    interleaved = grown.stream("added.between")  # and between draws
    interleaved.gauss(0, 1)
    rest = [loss.uniform(0, 5) for _ in range(10)]

    assert first + rest == expected


def test_stream_seeds_records_issued_streams_in_order() -> None:
    factory = RngFactory(42)
    assert dict(factory.stream_seeds) == {}
    factory.stream("rf.loss")
    factory.stream("env.sensor_noise")
    factory.stream("rf.loss")
    assert list(factory.stream_seeds) == ["rf.loss", "env.sensor_noise"]
    assert factory.stream_seeds["rf.loss"] == derive_seed(42, "rf.loss")
    with pytest.raises(TypeError):
        factory.stream_seeds["x"] = 1  # type: ignore[index]


def test_campaign_run_seeds_are_replayable_individually() -> None:
    campaign_seed = 1234
    seeds = [derive_seed(campaign_seed, f"run:{i}") for i in range(5)]
    assert derive_seed(campaign_seed, "run:3") == seeds[3]


def test_empty_stream_name_rejected() -> None:
    with pytest.raises(ValueError):
        RngFactory(1).stream("")
