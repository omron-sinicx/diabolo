"""Tests for parsing and creating Connect messages."""

from natnet.protocol import ConnectMessage, Version, deserialize, serialize  # noqa: F401


def test_parse_connect_packet_v3():
    """Test parsing a NatNet 3.0 packet containing a Connect message."""
    data = open('test_data/connect_packet_v3.bin', 'rb').read()
    info = deserialize(data, Version(3), strict=True)  # type: ConnectMessage

    assert info.payload == 'NatNetLib'
    assert info.version1 == Version(3)
    assert info.version2 == Version(3)


def test_parse_connect_packet_v2():
    """Test parsing a NatNet 2.10 packet containing a Connect message."""
    data = open('test_data/connect_packet_v2.bin', 'rb').read()
    info = deserialize(data, Version(3), strict=True)  # type: ConnectMessage

    assert info.payload == 'NatNetLib'
    assert info.version1 == Version(2, 10)
    assert info.version2 == Version(2, 10)


def test_serialize_connect_message():
    """Test serializing a Connect message."""
    expected = open('test_data/connect_packet_v3.bin', 'rb').read()
    actual = serialize(ConnectMessage('NatNetLib', Version(3), Version(3)))

    assert actual == expected
