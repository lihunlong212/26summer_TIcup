import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, UInt8
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from domain_bridge_pkg.domain_bridge import (
    DomainBridge,
    durable_qos,
    dynamic_tf_qos,
    static_tf_qos,
)


def test_fly_choice_coordinates_and_drone_state_cross_domains():
    rclpy.init(domain_id=99)
    bridge = DomainBridge()
    bridge_executor = SingleThreadedExecutor()
    bridge_executor.add_node(bridge)

    remote_context = Context()
    remote_context.init(domain_id=42)
    remote_probe = Node("domain_42_probe", context=remote_context)
    remote_executor = SingleThreadedExecutor(context=remote_context)
    remote_executor.add_node(remote_probe)

    choice_pub = remote_probe.create_publisher(UInt8, "/fly_choice", 10)
    received_states = []
    state_sub = remote_probe.create_subscription(
        UInt8,
        "/drone_state",
        lambda msg: received_states.append(int(msg.data)),
        durable_qos(),
    )
    received_coordinates = []
    coordinate_sub = remote_probe.create_subscription(
        Float32MultiArray,
        "/drone_position",
        lambda msg: received_coordinates.append(list(msg.data)),
        10,
    )
    received_dynamic_tf = []
    tf_sub = remote_probe.create_subscription(
        TFMessage,
        "/tf",
        lambda msg: received_dynamic_tf.extend(msg.transforms),
        dynamic_tf_qos(),
    )
    received_static_tf = []
    tf_static_sub = remote_probe.create_subscription(
        TFMessage,
        "/tf_static",
        lambda msg: received_static_tf.extend(msg.transforms),
        static_tf_qos(),
    )

    local_context = Context()
    local_context.init(domain_id=1)
    local_probe = Node("domain_1_probe", context=local_context)
    local_executor = SingleThreadedExecutor(context=local_context)
    local_executor.add_node(local_probe)
    received_choices = []
    choice_sub = local_probe.create_subscription(
        UInt8,
        "/fly_choice",
        lambda msg: received_choices.append(int(msg.data)),
        10,
    )
    state_pub = local_probe.create_publisher(
        UInt8, "/drone_state", durable_qos()
    )
    tf_broadcaster = TransformBroadcaster(local_probe)
    static_tf_broadcaster = StaticTransformBroadcaster(local_probe)

    assert state_sub is not None
    assert coordinate_sub is not None
    assert tf_sub is not None
    assert tf_static_sub is not None
    assert choice_sub is not None

    def publish_transform():
        transform = TransformStamped()
        transform.header.stamp = local_probe.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "laser_link"
        transform.transform.translation.x = 0.12
        transform.transform.translation.y = -0.34
        yaw_rad = math.radians(20.0)
        transform.transform.rotation.z = math.sin(yaw_rad / 2.0)
        transform.transform.rotation.w = math.cos(yaw_rad / 2.0)
        tf_broadcaster.sendTransform(transform)

    def publish_static_transform():
        transform = TransformStamped()
        transform.header.stamp = local_probe.get_clock().now().to_msg()
        transform.header.frame_id = "laser_link"
        transform.child_frame_id = "camera_link"
        transform.transform.rotation.w = 1.0
        static_tf_broadcaster.sendTransform(transform)

    def spin_all():
        local_executor.spin_once(timeout_sec=0.01)
        bridge_executor.spin_once(timeout_sec=0.01)
        remote_executor.spin_once(timeout_sec=0.01)

    try:
        # Position is available during mapping, but no state exists before the
        # local task controller publishes a value from 1 through 5.
        silence_deadline = time.monotonic() + 0.4
        while time.monotonic() < silence_deadline:
            publish_transform()
            publish_static_transform()
            spin_all()
        assert not received_states

        state_sent = False
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            choice = UInt8()
            choice.data = 1
            choice_pub.publish(choice)
            if not state_sent:
                state = UInt8()
                state.data = 3
                state_pub.publish(state)
                state_sent = True
            publish_transform()
            publish_static_transform()
            spin_all()
            if (
                1 in received_choices
                and received_states
                and received_coordinates
                and received_dynamic_tf
                and received_static_tf
            ):
                break

        assert 1 in received_choices
        assert received_states
        assert received_states[-1] == 3
        assert 0 not in received_states
        assert math.isclose(
            received_coordinates[-1][0], 12.0, abs_tol=0.2
        )
        assert math.isclose(
            received_coordinates[-1][1], -34.0, abs_tol=0.2
        )
        assert any(
            transform.header.frame_id == "map"
            and transform.child_frame_id == "laser_link"
            for transform in received_dynamic_tf
        )
        assert any(
            transform.header.frame_id == "laser_link"
            and transform.child_frame_id == "camera_link"
            for transform in received_static_tf
        )

        # One local transition is repeated by the bridge heartbeat.
        initial_count = len(received_states)
        heartbeat_deadline = time.monotonic() + 0.35
        while time.monotonic() < heartbeat_deadline:
            publish_transform()
            spin_all()
        assert len(received_states) >= initial_count + 2
        assert set(received_states) == {3}

        assert not remote_probe.get_publishers_info_by_topic(
            "/fleet/device_status"
        )
    finally:
        local_executor.remove_node(local_probe)
        local_probe.destroy_node()
        local_context.shutdown()
        remote_executor.remove_node(remote_probe)
        remote_probe.destroy_node()
        remote_context.shutdown()
        bridge_executor.remove_node(bridge)
        bridge.shutdown()
        bridge.destroy_node()
        rclpy.shutdown()
