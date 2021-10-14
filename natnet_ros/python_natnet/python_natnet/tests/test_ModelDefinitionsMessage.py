"""Tests for parsing ModelDefinitions messages."""

import pytest

from natnet.protocol import ModelDefinitionsMessage, Version, deserialize, serialize  # noqa: F401
from natnet.protocol.ModelDefinitionsMessage import (CameraDescription, MarkersetDescription,
                                                     RigidBodyDescription, SkeletonDescription)


def test_parse_modeldef_packet_v3():
    """Test parsing a NatNet 3.0 packet containing a ModelDefinitions message."""
    packet = open('test_data/modeldef_packet_v3.bin', 'rb').read()
    modeldef = deserialize(packet, Version(3), strict=True)  # type: ModelDefinitionsMessage

    # These values are verified against SampleClient where easy

    assert len(modeldef.models) == 3

    # RaceQuad rigid body description
    rb = modeldef.models[0]  # type:  RigidBodyDescription
    assert type(rb) == RigidBodyDescription
    assert rb.name == 'RaceQuad'
    assert rb.id_ == 2
    assert rb.parent_id == -1
    assert rb.offset_from_parent == (0.0, 0.0, 0.0)
    assert rb.marker_positions[0] == pytest.approx((0.16115899, 0.0350516, -0.0321813))
    assert rb.marker_positions[1] == pytest.approx((-0.030819, 0.0397819, -0.0219901))
    assert rb.marker_positions[2] == pytest.approx((0.18753099, -0.0333833, 0.148081))
    assert rb.marker_positions[3] == pytest.approx((-0.0748715, 6.68607e-05, 0.0329233))
    assert rb.marker_positions[4] == pytest.approx((-0.0580382, -0.0319419, -0.136807))

    # RaceQuad markerset definition
    ms1 = modeldef.models[1]  # type: MarkersetDescription
    assert type(ms1) == MarkersetDescription
    assert ms1.name == 'RaceQuad'
    assert ms1.marker_names == ['Marker1', 'Marker2', 'Marker3', 'Marker4', 'Marker5']

    # 'all' markerset definition
    ms2 = modeldef.models[2]  # type: MarkersetDescription
    assert type(ms2) == MarkersetDescription
    assert ms2.name == 'all'
    assert ms2.marker_names == ['RaceQuad_1', 'RaceQuad_2', 'RaceQuad_3', 'RaceQuad_4',
                                'RaceQuad_5']


def test_parse_modeldef_packet_v3_2():
    """Test parsing a NatNet 3.0 packet from Motive 2.1 containing mystery id=5 models."""
    packet = open('test_data/modeldef_packet_v3_2.bin', 'rb').read()
    modeldef = deserialize(packet, Version(3), strict=True)  # type: ModelDefinitionsMessage

    assert len(modeldef.models) == 7

    # Crossbow rigid body description
    rb = modeldef.models[0]  # type:  RigidBodyDescription
    assert type(rb) == RigidBodyDescription
    assert rb.name == 'crossbow'

    # Crossbow markerset definition
    ms1 = modeldef.models[1]  # type: MarkersetDescription
    assert type(ms1) == MarkersetDescription
    assert ms1.name == 'crossbow'

    # 'all' markerset definition
    ms2 = modeldef.models[2]  # type: MarkersetDescription
    assert type(ms2) == MarkersetDescription
    assert ms2.name == 'all'

    # Camera definitions
    cam1 = modeldef.models[3]  # type: CameraDescription
    assert type(cam1) == CameraDescription
    assert cam1.name == "Prime 13 #28108"

    cam2 = modeldef.models[4]  # type: CameraDescription
    assert type(cam1) == CameraDescription
    assert cam2.name == "Prime 13 #28107"

    cam3 = modeldef.models[5]  # type: CameraDescription
    assert type(cam1) == CameraDescription
    assert cam3.name == "Prime 13 #28105"

    cam4 = modeldef.models[6]  # type: CameraDescription
    assert type(cam1) == CameraDescription
    assert cam4.name == "Prime 13 #28106"


def test_parse_modeldef_packet_v2():
    """Test parsing a NatNet 2.10 packet containing a ModelDefinitions message."""
    packet = open('test_data/modeldef_packet_v2.bin', 'rb').read()
    modeldef = deserialize(packet, Version(2, 10), strict=True)  # type: ModelDefinitionsMessage

    # TODO: Verify against SampleClient

    assert len(modeldef.models) == 5

    # RigidBody 1 rigid body description
    rb = modeldef.models[0]  # type: RigidBodyDescription
    assert type(rb) == RigidBodyDescription
    assert rb.name == 'RigidBody 1'
    assert rb.id_ == 1
    assert rb.parent_id == -1
    assert rb.offset_from_parent == (0.0, 0.0, 0.0)

    # RigidBody 1 markerset description
    ms1 = modeldef.models[1]  # type: MarkersetDescription
    assert type(ms1) == MarkersetDescription
    assert ms1.name == 'RigidBody 1'
    assert ms1.marker_names == ['Marker1', 'Marker2', 'Marker3']

    # Karlie skeleton description
    sd = modeldef.models[2]  # type: SkeletonDescription
    assert type(sd) == SkeletonDescription
    assert sd.name == 'Karlie'
    assert sd.id_ == 4
    assert len(sd.rigid_bodies) == 21
    assert [body.name for body in sd.rigid_bodies] == [
        'Karlie_Hip', 'Karlie_Ab', 'Karlie_Chest', 'Karlie_Neck', 'Karlie_Head', 'Karlie_LShoulder',
        'Karlie_LUArm', 'Karlie_LFArm', 'Karlie_LHand', 'Karlie_RShoulder', 'Karlie_RUArm',
        'Karlie_RFArm', 'Karlie_RHand', 'Karlie_LThigh', 'Karlie_LShin', 'Karlie_LFoot',
        'Karlie_RThigh', 'Karlie_RShin', 'Karlie_RFoot', 'Karlie_LToe', 'Karlie_RToe']

    # Karlie markerset description
    ms2 = modeldef.models[3]  # type: MarkersetDescription
    assert type(ms2) == MarkersetDescription
    assert ms2.name == 'Karlie'
    assert ms2.marker_names == [
        'Karlie_WaistLFront', 'Karlie_WaistRFront', 'Karlie_WaistLBack', 'Karlie_WaistRBack',
        'Karlie_BackTop', 'Karlie_Chest', 'Karlie_BackLeft', 'Karlie_BackRight', 'Karlie_HeadTop',
        'Karlie_HeadFront', 'Karlie_HeadSide', 'Karlie_LShoulderBack', 'Karlie_LShoulderTop',
        'Karlie_LElbowOut', 'Karlie_LUArmHigh', 'Karlie_LHandOut', 'Karlie_LWristOut',
        'Karlie_LWristIn', 'Karlie_RShoulderBack', 'Karlie_RShoulderTop', 'Karlie_RElbowOut',
        'Karlie_RUArmHigh', 'Karlie_RHandOut', 'Karlie_RWristOut', 'Karlie_RWristIn',
        'Karlie_LKneeOut', 'Karlie_LThigh', 'Karlie_LAnkleOut', 'Karlie_LShin', 'Karlie_LToeOut',
        'Karlie_LToeIn', 'Karlie_RKneeOut', 'Karlie_RThigh', 'Karlie_RAnkleOut', 'Karlie_RShin',
        'Karlie_RToeOut', 'Karlie_RToeIn']

    # 'all' markerset definition
    ms3 = modeldef.models[4]  # type: MarkersetDescription
    assert type(ms3) == MarkersetDescription
    assert ms3.name == 'all'
    assert ms3.marker_names == [
        'RigidBody 1_1', 'RigidBody 1_2', 'RigidBody 1_3', 'Karlie_WaistLFront',
        'Karlie_WaistRFront', 'Karlie_WaistLBack', 'Karlie_WaistRBack', 'Karlie_BackTop',
        'Karlie_Chest', 'Karlie_BackLeft', 'Karlie_BackRight', 'Karlie_HeadTop', 'Karlie_HeadFront',
        'Karlie_HeadSide', 'Karlie_LShoulderBack', 'Karlie_LShoulderTop', 'Karlie_LElbowOut',
        'Karlie_LUArmHigh', 'Karlie_LHandOut', 'Karlie_LWristOut', 'Karlie_LWristIn',
        'Karlie_RShoulderBack', 'Karlie_RShoulderTop', 'Karlie_RElbowOut', 'Karlie_RUArmHigh',
        'Karlie_RHandOut', 'Karlie_RWristOut', 'Karlie_RWristIn', 'Karlie_LKneeOut',
        'Karlie_LThigh', 'Karlie_LAnkleOut', 'Karlie_LShin', 'Karlie_LToeOut', 'Karlie_LToeIn',
        'Karlie_RKneeOut', 'Karlie_RThigh', 'Karlie_RAnkleOut', 'Karlie_RShin', 'Karlie_RToeOut',
        'Karlie_RToeIn']


def test_parse_modeldef_packet_skeleton_v3():
    """Test parsing a NatNet 3.0 packet containing a ModelDefinitions message with skeletons."""
    packet = open('test_data/modeldef_packet_skeleton_v3.bin', 'rb').read()
    modeldef = deserialize(packet, Version(3), strict=True)  # type: ModelDefinitionsMessage

    # TODO: Verify against SampleClient

    assert len(modeldef.models) == 3

    # Skeleton description
    sd = modeldef.models[0]  # type: SkeletonDescription
    assert type(sd) == SkeletonDescription
    assert sd.name == 'Skeleton 002'
    assert sd.id_ == 3
    assert len(sd.rigid_bodies) == 21
    assert [body.name for body in sd.rigid_bodies] == [
        'Skeleton 002_Hip', 'Skeleton 002_Ab', 'Skeleton 002_Chest', 'Skeleton 002_Neck',
        'Skeleton 002_Head', 'Skeleton 002_LShoulder', 'Skeleton 002_LUArm', 'Skeleton 002_LFArm',
        'Skeleton 002_LHand', 'Skeleton 002_RShoulder', 'Skeleton 002_RUArm', 'Skeleton 002_RFArm',
        'Skeleton 002_RHand', 'Skeleton 002_LThigh', 'Skeleton 002_LShin', 'Skeleton 002_LFoot',
        'Skeleton 002_RThigh', 'Skeleton 002_RShin', 'Skeleton 002_RFoot', 'Skeleton 002_LToe',
        'Skeleton 002_RToe']

    # Skeleton markerset description
    ms = modeldef.models[1]  # type: MarkersetDescription
    assert type(ms) == MarkersetDescription
    assert ms.name == 'Skeleton 002'
    assert ms.marker_names == [
        'WaistLFront', 'WaistRFront', 'WaistLBack', 'WaistRBack', 'BackTop', 'Chest', 'BackLeft',
        'BackRight', 'HeadTop', 'HeadFront', 'HeadSide', 'LShoulderBack', 'LShoulderTop',
        'LElbowOut', 'LUArmHigh', 'LHandOut', 'LWristOut', 'LWristIn', 'RShoulderBack',
        'RShoulderTop', 'RElbowOut', 'RUArmHigh', 'RHandOut', 'RWristOut', 'RWristIn',
        'LKneeOut', 'LThigh', 'LAnkleOut', 'LShin', 'LToeOut', 'LToeIn', 'RKneeOut', 'RThigh',
        'RAnkleOut', 'RShin', 'RToeOut', 'RToeIn']

    # 'all' markerset definition
    ms2 = modeldef.models[2]  # type: MarkersetDescription
    assert type(ms2) == MarkersetDescription
    assert ms2.name == 'all'
    assert ms2.marker_names == ms.marker_names


def test_serialize_modeldef_message():
    """Test serializing a ModelDefinitionsMessage."""
    packet = open('test_data/modeldef_packet_v3.bin', 'rb').read()

    msg = ModelDefinitionsMessage(
        models=[
            RigidBodyDescription(
                name='RaceQuad',
                id_=2,
                parent_id=-1,
                offset_from_parent=(0.0, 0.0, 0.0),
                marker_positions=[
                    (0.1611589938402176, 0.03505159914493561, -0.03218130022287369),
                    (-0.0308190006762743, 0.03978189826011658, -0.021990099921822548),
                    (0.1875309944152832, -0.03338329866528511, 0.1480810046195984),
                    (-0.0748715028166771, 6.686070264549926e-05, 0.03292329981923103),
                    (-0.05803820118308067, -0.03194189816713333, -0.13680699467658997)],
                required_active_labels=[0, 0, 0, 0, 0]
            ),
            MarkersetDescription(
                name='RaceQuad',
                marker_names=['Marker1', 'Marker2', 'Marker3', 'Marker4', 'Marker5']
            ),
            MarkersetDescription(
                name='all',
                marker_names=['RaceQuad_1', 'RaceQuad_2', 'RaceQuad_3', 'RaceQuad_4', 'RaceQuad_5']
            )
        ]
    )
    serialized_msg = serialize(msg)
    print(len(serialized_msg), len(packet))
    assert serialized_msg == packet
