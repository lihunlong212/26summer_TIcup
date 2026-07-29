from __future__ import annotations

import json
import math
import threading
from typing import Any

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from std_msgs.msg import Bool, Int16, Int32, String, UInt8
from tf2_ros import Buffer, TransformException, TransformListener


def durable_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class DomainBridge(Node):
    """Bridges flight selection into Domain 1 and telemetry back to Domain 10."""

    def __init__(self) -> None:
        super().__init__("domain_bridge")
        self._local_domain_id = int(
            self.declare_parameter("local_domain_id", 1).value
        )
        self._map_frame = str(self.declare_parameter("map_frame", "map").value)
        self._robot_frame = str(
            self.declare_parameter("robot_frame", "laser_link").value
        )
        status_frequency_hz = max(
            1.0, float(self.declare_parameter("status_frequency_hz", 10.0).value)
        )

        self._lock = threading.RLock()
        self._height_cm: int | None = None
        self._mission_state = "WAITING_ROUTE"
        self._waypoint_index = -1
        self._fly_choice = 0
        self._vision_active = False
        self._vision_fresh = False

        self._status_pub = self.create_publisher(String, "/fleet/device_status", 10)
        self._fly_choice_sub = self.create_subscription(
            UInt8,
            "/fly_choice",
            self._fly_choice_from_domain_10,
            10,
        )

        self._local_context = Context()
        self._local_context.init(domain_id=self._local_domain_id)
        self._local_node = Node(
            "domain_bridge_local_endpoint", context=self._local_context
        )
        self._local_fly_choice_pub = self._local_node.create_publisher(
            UInt8, "/fly_choice", 10
        )
        self._local_subscriptions = [
            self._local_node.create_subscription(
                Int16, "/height", self._on_height, 10
            ),
            self._local_node.create_subscription(
                String,
                "/mission_state",
                self._on_mission_state,
                durable_qos(),
            ),
            self._local_node.create_subscription(
                Int32,
                "/current_waypoint_index",
                self._on_waypoint_index,
                durable_qos(),
            ),
            self._local_node.create_subscription(
                UInt8,
                "/fly_choice_status",
                self._on_fly_choice_status,
                durable_qos(),
            ),
            self._local_node.create_subscription(
                Bool,
                "/visual_takeover_active",
                self._on_visual_active,
                durable_qos(),
            ),
            self._local_node.create_subscription(
                Bool,
                "/vision_fresh",
                self._on_vision_fresh,
                durable_qos(),
            ),
        ]
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer, self._local_node, spin_thread=False
        )
        self._local_executor = SingleThreadedExecutor(context=self._local_context)
        self._local_executor.add_node(self._local_node)
        self._local_thread = threading.Thread(
            target=self._spin_local, name="domain_1_bridge", daemon=True
        )
        self._local_thread.start()

        self._status_timer = self.create_timer(
            1.0 / status_frequency_hz, self._publish_status
        )
        self.get_logger().info(
            f"bridge ready: current domain -> local DOMAIN={self._local_domain_id}"
        )

    def _spin_local(self) -> None:
        try:
            self._local_executor.spin()
        except Exception as error:  # pragma: no cover - runtime protection
            self.get_logger().error(f"local-domain executor stopped: {error}")

    def _fly_choice_from_domain_10(self, msg: UInt8) -> None:
        if msg.data not in (1, 2):
            self.get_logger().warning(
                f"ignoring invalid DOMAIN-10 /fly_choice={msg.data}"
            )
            return
        self._local_fly_choice_pub.publish(msg)
        self.get_logger().info(
            f"bridged /fly_choice={msg.data} to DOMAIN={self._local_domain_id}"
        )

    def _on_height(self, msg: Int16) -> None:
        with self._lock:
            self._height_cm = int(msg.data)

    def _on_mission_state(self, msg: String) -> None:
        with self._lock:
            self._mission_state = msg.data

    def _on_waypoint_index(self, msg: Int32) -> None:
        with self._lock:
            self._waypoint_index = int(msg.data)

    def _on_fly_choice_status(self, msg: UInt8) -> None:
        with self._lock:
            self._fly_choice = int(msg.data)

    def _on_visual_active(self, msg: Bool) -> None:
        with self._lock:
            self._vision_active = bool(msg.data)

    def _on_vision_fresh(self, msg: Bool) -> None:
        with self._lock:
            self._vision_fresh = bool(msg.data)

    @staticmethod
    def _yaw_degrees(rotation: Any) -> float:
        sin_yaw = 2.0 * (
            float(rotation.w) * float(rotation.z)
            + float(rotation.x) * float(rotation.y)
        )
        cos_yaw = 1.0 - 2.0 * (
            float(rotation.y) ** 2 + float(rotation.z) ** 2
        )
        return math.degrees(math.atan2(sin_yaw, cos_yaw))

    def _pose(self) -> tuple[float | None, float | None, float | None]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame, self._robot_frame, Time()
            )
        except TransformException:
            return None, None, None
        translation = transform.transform.translation
        return (
            round(float(translation.x) * 100.0, 2),
            round(float(translation.y) * 100.0, 2),
            round(self._yaw_degrees(transform.transform.rotation), 2),
        )

    def _publish_status(self) -> None:
        x_cm, y_cm, yaw_deg = self._pose()
        with self._lock:
            payload = {
                "x_cm": x_cm,
                "y_cm": y_cm,
                "z_cm": self._height_cm,
                "yaw_deg": yaw_deg,
                "fly_choice": self._fly_choice,
                "current_waypoint_index": self._waypoint_index,
                "mission_state": self._mission_state,
                "vision_active": self._vision_active,
                "vision_fresh": self._vision_fresh,
            }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self._status_pub.publish(msg)

    def shutdown(self) -> None:
        try:
            self._local_executor.shutdown()
        except Exception:
            pass
        self._local_thread.join(timeout=2.0)
        try:
            self._local_node.destroy_node()
        except Exception:
            pass
        try:
            self._local_context.shutdown()
        except Exception:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DomainBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
