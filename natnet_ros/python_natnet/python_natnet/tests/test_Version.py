"""Tests for Version class."""

from natnet import Version
from natnet.protocol.common import ParseBuffer


def test_construct_default():
    """Test constructing with defaults."""
    version = Version(3)
    assert version.major == 3
    assert version.minor == 0
    assert version.build == 0
    assert version.revision == 0


def test_construct():
    """Test constructing with arguments."""
    version = Version(1, 2, 3, 4)
    assert version.major == 1
    assert version.minor == 2
    assert version.build == 3
    assert version.revision == 4


def test_comparisons_from_protocol_3_0():
    """Test that each comparison used in the parsers works for version 3.0."""
    version = Version(3)
    assert version > Version(2)
    assert version >= Version(2)
    assert version >= Version(2, 3)
    assert version >= Version(2, 6)
    assert version >= Version(2, 9)
    assert version >= Version(2, 11)
    assert version >= Version(3)
    assert not version < Version(3)


def test_deserialize():
    assert Version.deserialize(ParseBuffer(b'\x04\x03\x02\x01')) == Version(4, 3, 2, 1)


def test_serialize():
    assert Version(4, 3, 2, 1).serialize() == b'\x04\x03\x02\x01'


def test_construct_and_compare_versions(benchmark):
    """Benchmark constructing a Version and comparing it to a previously constructed Version."""
    v = Version(3.0)

    def work(version):
        _ = version >= Version(2, 11)  # noqa: F841

    benchmark.pedantic(work, args=(v,), iterations=100, rounds=100)
