"""Tests for parsing and creating Discovery messages."""

from natnet.protocol import DiscoveryMessage, Version, deserialize, serialize  # noqa: F401


def test_parse_discovery_packet_v3():
    """Test parsing a NatNet 3.0 packet containing a Discovery message."""
    data = open('test_data/discovery_packet_v3.bin', 'rb').read()
    info = deserialize(data, Version(3), strict=True)  # type: DiscoveryMessage

    assert info.payload == 'NatNetLib'
    assert info.version1 == Version(3)
    assert info.version2 == Version(3)


def test_serialize_discovery_message():
    """Test serializing a Discovery message."""
    expected = open('test_data/discovery_packet_v3.bin', 'rb').read()
    actual = serialize(DiscoveryMessage('NatNetLib', Version(3), Version(3)))

    assert actual == expected
