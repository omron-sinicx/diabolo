"""Tests for parsing MocapFrame messages."""

import pytest

from natnet.protocol import MocapFrameMessage, Version, deserialize, serialize
from natnet.protocol.common import ParseBuffer
from natnet.protocol.MocapFrameMessage import LabelledMarker, Markerset, RigidBody, TimingInfo


def test_parse_mocapframe_packet_v3():
    """Test parsing a NatNet 3.0 packet containing a MocapFrame."""
    packet = open('test_data/mocapframe_packet_v3.bin', 'rb').read()
    frame = deserialize(packet, Version(3), strict=True)  # type: MocapFrameMessage

    # These values are verified against SampleClient where easy

    assert frame.frame_number == 162734
    assert len(frame.markersets) == 0

    assert len(frame.rigid_bodies) == 1
    body = frame.rigid_bodies[0]
    assert body.id_ == 2
    assert body.position == pytest.approx((0.1744466, 1.4471314, -0.7343040))
    assert body.orientation == pytest.approx((-0.05459423, 0.5099482, 0.04370357, -0.8573577))
    assert body.mean_error == pytest.approx(0.0005152203)
    assert body.tracking_valid

    assert len(frame.skeletons) == 0

    assert len(frame.labelled_markers) == 6
    marker0 = frame.labelled_markers[0]  # type: LabelledMarker
    assert marker0.model_id == 2
    assert marker0.marker_id == 1
    assert marker0.position == pytest.approx((0.1272162, 1.5050275, -0.8858284))
    assert marker0.size == pytest.approx(0.02143982)
    assert marker0._params == 10
    assert not marker0.occluded
    assert marker0.point_cloud_solved
    assert not marker0.model_solved
    assert marker0.has_model
    assert not marker0.unlabelled
    assert not marker0.active
    assert marker0.residual == pytest.approx(0.0002074828)
    # Assume markers 1-4 are correct if 0 and 5 are
    marker5 = frame.labelled_markers[5]
    assert marker5.model_id == 0
    assert marker5.marker_id == 50007
    assert marker5.position == pytest.approx((0.1708117, 1.5076591, -0.8402346))
    assert marker5.size == pytest.approx(0.02015734)
    assert marker5._params == 18
    assert not marker5.occluded
    assert marker5.point_cloud_solved
    assert not marker5.model_solved
    assert not marker5.has_model
    assert marker5.unlabelled
    assert not marker5.active
    assert marker5.residual == pytest.approx(0.0005593782)

    assert len(frame.force_plates) == 0
    assert len(frame.devices) == 0

    assert frame.timing_info.timecode == 0
    assert frame.timing_info.timecode_subframe == 0
    assert frame.timing_info.timestamp == pytest.approx(1356.117)
    assert frame.timing_info.camera_mid_exposure_timestamp == 1416497730518
    assert frame.timing_info.camera_data_received_timestamp == 1416497745808
    assert frame.timing_info.transmit_timestamp == 1416497748722

    assert not frame.is_recording
    assert not frame.tracked_models_changed


def test_parse_mocapframe_packet_v2():
    """Test parsing a NatNet 2.10 packet containing a MocapFrame."""
    # Packet 778 from Omar's data
    packet = open('test_data/mocapframe_packet_v2.bin', 'rb').read()
    frame = deserialize(packet, Version(2, 10), strict=True)  # type: MocapFrameMessage

    # These values are verified against SampleClient where easy

    assert frame.frame_number == 109238
    assert len(frame.markersets) == 3
    assert frame.markersets[0].name == 'RigidBody 1'
    assert frame.markersets[1].name == 'Karlie'
    assert frame.markersets[2].name == 'all'

    assert len(frame.rigid_bodies) == 1
    body = frame.rigid_bodies[0]
    assert body.id_ == 1
    assert body.position == pytest.approx((1.38, 1.21, 1.62), abs=0.005)
    assert body.orientation == pytest.approx((-0.74, 0.51, 0.12, -0.42), abs=0.005)
    assert body.mean_error == pytest.approx(0.0)
    assert not body.tracking_valid

    assert len(frame.skeletons) == 1
    skeleton = frame.skeletons[0]
    assert skeleton.id_ == 4
    assert len(skeleton.rigid_bodies) == 21
    bone0 = skeleton.rigid_bodies[0]
    assert bone0.id_ == 262145
    assert bone0.position == pytest.approx((0.50, 0.85, 0.43), abs=0.005)
    assert bone0.orientation == pytest.approx((0.02, 0.80, 0.01, 0.60), abs=0.005)
    # skip 19 bones
    bone20 = skeleton.rigid_bodies[20]
    assert bone20.id_ == 262165
    assert bone20.position == pytest.approx((-0.00, -0.06, 0.13), abs=0.005)
    assert bone20.orientation == pytest.approx((-0.00, 0.00, 0.00, -1.00), abs=0.005)

    assert len(frame.labelled_markers) == 44
    marker0 = frame.labelled_markers[0]
    assert marker0.model_id == 4
    assert marker0.marker_id == 5
    assert marker0.position == pytest.approx((0.54, 0.93, 0.28), abs=0.005)
    assert marker0.size == pytest.approx(0.02, abs=0.005)
    assert not marker0.occluded
    assert marker0.point_cloud_solved
    assert not marker0.model_solved
    # Skip 42 markers
    marker43 = frame.labelled_markers[43]
    assert marker43.model_id == 0
    assert marker43.marker_id == 7637
    assert marker43.position == pytest.approx((1.13, 1.48, 1.42), abs=0.005)
    assert marker43.size == pytest.approx(0.01, abs=0.005)
    # assert marker5._params == 18
    assert not marker43.occluded
    assert marker43.point_cloud_solved
    assert not marker43.model_solved

    assert len(frame.force_plates) == 0
    assert len(frame.devices) == 0

    assert frame.timing_info.timecode == 0
    assert frame.timing_info.timecode_subframe == 0
    assert frame.timing_info.timestamp == pytest.approx(910.32, abs=0.005)

    assert not frame.is_recording
    assert not frame.tracked_models_changed


def test_serialize_mocapframe_message():
    """Test serializing a MocapFrameMessage."""
    packet = open('test_data/mocapframe_packet_v3.bin', 'rb').read()

    rigid_body = RigidBody(
        id_=2, position=(0.17444664239883423, 1.4471313953399658, -0.7343040108680725),
        orientation=(-0.05459423363208771, 0.509948194026947, 0.04370357096195221, -0.8573576807975769),
        mean_error=0.0005152203375473619, params=1
    )
    markers = [
        LabelledMarker(
            model_id=2, marker_id=1,
            position=(0.12721621990203857, 1.5050275325775146, -0.8858283758163452),
            size=0.021439820528030396, params=10, residual=0.00020748283714056015),
        LabelledMarker(
            model_id=2, marker_id=2,
            position=(0.20898520946502686, 1.4832806587219238, -0.7121866345405579),
            size=0.020884789526462555, params=10, residual=0.000539067666977644),
        LabelledMarker(
            model_id=2, marker_id=3,
            position=(-0.04574164003133774, 1.4310429096221924, -0.8313022255897522),
            size=0.020809074863791466, params=10, residual=0.000689117528963834),
        LabelledMarker(
            model_id=2, marker_id=4,
            position=(0.18133828043937683, 1.4358338117599487, -0.6535942554473877),
            size=0.019269507378339767, params=10, residual=0.0003837273397948593),
        LabelledMarker(
            model_id=2, marker_id=5,
            position=(0.32139891386032104, 1.4146512746810913, -0.7529537677764893),
            size=0.021224740892648697, params=10, residual=0.0006357387755997479),
        LabelledMarker(
            model_id=0, marker_id=50007,
            position=(0.17081165313720703, 1.5076590776443481, -0.840234637260437),
            size=0.0201573446393013, params=18, residual=0.0005593782989308238)
    ]
    timing_info = TimingInfo(
        timecode=0,
        timecode_subframe=0,
        timestamp=1356.1166666666666,
        camera_mid_exposure_timestamp=1416497730518,
        camera_data_received_timestamp=1416497745808,
        transmit_timestamp=1416497748722
    )
    msg = MocapFrameMessage(
        frame_number=162734,
        markersets=[],
        rigid_bodies=[rigid_body],
        skeletons=[],
        labelled_markers=markers,
        force_plates=[],
        devices=[],
        timing_info=timing_info,
        params=0
    )
    old_serialize = msg.serialize
    msg.serialize = lambda: old_serialize(include_unlabelled=True)
    serialized_msg = serialize(msg)
    print(len(serialized_msg), len(packet))
    assert serialized_msg == packet


def test_serialize_and_deserialize_markerset():
    """Just need something that tests ParseBuffer.unpack_cstr without a size."""
    markerset = Markerset('test', [(1.0, 2.0, 3.0)])
    packet = ParseBuffer(markerset.serialize())
    assert Markerset.deserialize(packet, Version(3)) == markerset


def test_deserialize_mocapframe(benchmark):
    """Benchmark parsing a NatNet 3.0 packet containing a MocapFrame."""
    packet = open('test_data/mocapframe_packet_v3.bin', 'rb').read()
    benchmark(deserialize, packet, Version(3))
