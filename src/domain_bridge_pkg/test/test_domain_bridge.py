import json
import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Int16, Int32, String, UInt8
from tf2_ros import TransformBroadcaster

from domain_bridge_pkg.domain_bridge import DomainBridge, durable_qos


def test_fly_choice_and_status_cross_domains():
    rclpy.init(domain_id=10)
    bridge = DomainBridge()
    domain_10_probe = Node("domain_10_probe")
    domain_10_executor = SingleThreadedExecutor()
    domain_10_executor.add_node(bridge)
    domain_10_executor.add_node(domain_10_probe)

    choice_pub = domain_10_probe.create_publisher(UInt8, "/fly_choice", 10)
    received_status = []
    status_sub = domain_10_probe.create_subscription(
        String,
        "/fleet/device_status",
        lambda msg: received_status.append(json.loads(msg.data)),
        10,
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
    height_pub = local_probe.create_publisher(Int16, "/height", 10)
    state_pub = local_probe.create_publisher(
        String, "/mission_state", durable_qos()
    )
    index_pub = local_probe.create_publisher(
        Int32, "/current_waypoint_index", durable_qos()
    )
    choice_status_pub = local_probe.create_publisher(
        UInt8, "/fly_choice_status", durable_qos()
    )
    visual_pub = local_probe.create_publisher(
        Bool, "/visual_takeover_active", durable_qos()
    )
    fresh_pub = local_probe.create_publisher(
        Bool, "/vision_fresh", durable_qos()
    )
    tf_broadcaster = TransformBroadcaster(local_probe)

    # Retain subscriptions explicitly for the duration of the test.
    assert status_sub is not None
    assert choice_sub is not None

    try:
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            choice = UInt8()
            choice.data = 1
            choice_pub.publish(choice)

            height = Int16()
            height.data = 123
            height_pub.publish(height)
            state = String()
            state.data = "FOLLOW_DROP"
            state_pub.publish(state)
            index = Int32()
            index.data = 3
            index_pub.publish(index)
            choice_status = UInt8()
            choice_status.data = 1
            choice_status_pub.publish(choice_status)
            active = Bool()
            active.data = True
            visual_pub.publish(active)
            fresh = Bool()
            fresh.data = True
            fresh_pub.publish(fresh)

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

            local_executor.spin_once(timeout_sec=0.01)
            domain_10_executor.spin_once(timeout_sec=0.01)
            if received_choices and received_status:
                payload = received_status[-1]
                if (
                    payload.get("fly_choice") == 1
                    and payload.get("mission_state") == "FOLLOW_DROP"
                    and payload.get("z_cm") == 123
                ):
                    break

        assert 1 in received_choices
        assert received_status
        payload = received_status[-1]
        assert "device_id" not in payload
        assert "route_choice" not in payload
        assert payload["fly_choice"] == 1
        assert payload["current_waypoint_index"] == 3
        assert payload["mission_state"] == "FOLLOW_DROP"
        assert payload["vision_active"] is True
        assert payload["vision_fresh"] is True
        assert payload["z_cm"] == 123
        assert math.isclose(payload["x_cm"], 12.0, abs_tol=0.2)
        assert math.isclose(payload["y_cm"], -34.0, abs_tol=0.2)
        assert math.isclose(payload["yaw_deg"], 20.0, abs_tol=0.2)
    finally:
        local_executor.remove_node(local_probe)
        local_probe.destroy_node()
        local_context.shutdown()
        domain_10_executor.remove_node(domain_10_probe)
        domain_10_executor.remove_node(bridge)
        domain_10_probe.destroy_node()
        bridge.shutdown()
        bridge.destroy_node()
        rclpy.shutdown()
