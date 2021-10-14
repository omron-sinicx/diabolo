"""Tests for parsing and creating RequestModelDefitions messages."""

from natnet.protocol import RequestModelDefinitionsMessage, Version, deserialize, serialize


def test_parse_requestmodeldef_packet_v3():
    """Test parsing a NatNet 3.0 packet containing a RequestModelDefinitions message."""
    data = open('test_data/requestmodeldef_packet_v3.bin', 'rb').read()
    deserialize(data, Version(3), strict=True)

    # No payload


def test_parse_requestmodeldef_packet_v2():
    """Test parsing a NatNet 2.10 packet containing a RequestModelDefinitions message."""
    data = open('test_data/requestmodeldef_packet_v2.bin', 'rb').read()
    deserialize(data, Version(2, 10), strict=True)

    # No payload


def test_serialize_requestmodeldef_message():
    """Test serializing a RequestModelDefinitions message."""
    expected = open('test_data/requestmodeldef_packet_v3.bin', 'rb').read()
    actual = serialize(RequestModelDefinitionsMessage())

    assert actual == expected


def test_requestmodeldef_equal():
    assert RequestModelDefinitionsMessage() == RequestModelDefinitionsMessage()
