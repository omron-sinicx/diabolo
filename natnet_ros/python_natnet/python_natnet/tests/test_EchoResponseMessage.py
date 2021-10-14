"""Tests for parsing and creating EchoResponse messages."""

from natnet.protocol import EchoResponseMessage, Version, deserialize, serialize  # noqa: F401


def test_parse_echoresponse_packet_v3():
    """Test parsing a NatNet 3.0 packet containing an EchoResponse message."""
    packet = open('test_data/echoresponse_packet_v3.bin', 'rb').read()
    echo_response = deserialize(packet, Version(3), strict=True)  # type: EchoResponseMessage

    assert echo_response.request_timestamp == 278554190
    assert echo_response.received_timestamp == 1416498545924


def test_serialize_echoresponse_message():
    """Test serializing an EchoResponse message."""
    expected = open('test_data/echoresponse_packet_v3.bin', 'rb').read()
    actual = serialize(EchoResponseMessage(278554190, 1416498545924))

    assert actual == expected


def test_deserialize_echoresponse(benchmark):
    """Benchmark parsing a NatNet 3.0 packet containing an EchoResponse message."""
    packet = open('test_data/echoresponse_packet_v3.bin', 'rb').read()
    benchmark(deserialize, packet, Version(3))
