from __future__ import annotations

import threading

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
from std_msgs.msg import Float32MultiArray, UInt8
from tf2_ros import Buffer, TransformException, TransformListener


def durable_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class DomainBridge(Node):
    """Bridge flight choice, position, and compact state across ROS domains."""

    def __init__(self) -> None:
        super().__init__("domain_bridge")
        self._local_domain_id = int(
            self.declare_parameter("local_domain_id", 1).value
        )
        self._remote_domain_id = int(
            self.declare_parameter("remote_domain_id", 42).value
        )
        self._fly_choice_topic = str(
            self.declare_parameter("fly_choice_topic", "/fly_choice").value
        )
        self._coordinate_topic = str(
            self.declare_parameter(
                "coordinate_topic", "/drone_position"
            ).value
        )
        self._drone_state_topic = str(
            self.declare_parameter(
                "drone_state_topic", "/drone_state"
            ).value
        )
        self._map_frame = str(self.declare_parameter("map_frame", "map").value)
        self._robot_frame = str(
            self.declare_parameter("robot_frame", "laser_link").value
        )
        publish_frequency_hz = max(
            1.0,
            float(
                self.declare_parameter(
                    "publish_frequency_hz", 10.0
                ).value
            ),
        )

        self._lock = threading.RLock()
        self._drone_state: int | None = None

        self._local_context = Context()
        self._local_context.init(domain_id=self._local_domain_id)
        self._local_node = Node(
            "domain_bridge_local_endpoint", context=self._local_context
        )
        self._local_fly_choice_pub = self._local_node.create_publisher(
            UInt8, "/fly_choice", 10
        )
        self._local_drone_state_sub = (
            self._local_node.create_subscription(
                UInt8,
                "/drone_state",
                self._on_drone_state,
                durable_qos(),
            )
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer, self._local_node, spin_thread=False
        )
        self._local_executor = SingleThreadedExecutor(
            context=self._local_context
        )
        self._local_executor.add_node(self._local_node)

        self._remote_context = Context()
        self._remote_context.init(domain_id=self._remote_domain_id)
        self._remote_node = Node(
            "domain_bridge_remote_endpoint", context=self._remote_context
        )
        self._coordinate_pub = self._remote_node.create_publisher(
            Float32MultiArray, self._coordinate_topic, 10
        )
        self._drone_state_pub = self._remote_node.create_publisher(
            UInt8, self._drone_state_topic, durable_qos()
        )
        self._remote_fly_choice_sub = (
            self._remote_node.create_subscription(
                UInt8,
                self._fly_choice_topic,
                self._fly_choice_from_remote,
                10,
            )
        )
        self._remote_executor = SingleThreadedExecutor(
            context=self._remote_context
        )
        self._remote_executor.add_node(self._remote_node)

        self._local_thread = threading.Thread(
            target=self._spin_local,
            name=f"domain_{self._local_domain_id}_bridge",
            daemon=True,
        )
        self._remote_thread = threading.Thread(
            target=self._spin_remote,
            name=f"domain_{self._remote_domain_id}_bridge",
            daemon=True,
        )
        self._local_thread.start()
        self._remote_thread.start()

        self._publish_timer = self._remote_node.create_timer(
            1.0 / publish_frequency_hz, self._publish_telemetry
        )
        self.get_logger().info(
            "bridge ready: "
            f"local DOMAIN={self._local_domain_id} <-> "
            f"remote DOMAIN={self._remote_domain_id}; "
            f"coordinates={self._coordinate_topic}; "
            f"state={self._drone_state_topic}"
        )

    def _spin_local(self) -> None:
        try:
            self._local_executor.spin()
        except Exception as error:  # pragma: no cover - runtime protection
            if self.context.ok():
                self.get_logger().error(
                    f"local-domain executor stopped: {error}"
                )

    def _spin_remote(self) -> None:
        try:
            self._remote_executor.spin()
        except Exception as error:  # pragma: no cover - runtime protection
            if self.context.ok():
                self.get_logger().error(
                    f"remote-domain executor stopped: {error}"
                )

    def _fly_choice_from_remote(self, msg: UInt8) -> None:
        if msg.data not in (1, 2):
            self.get_logger().warning(
                f"ignoring invalid DOMAIN-{self._remote_domain_id} "
                f"{self._fly_choice_topic}={msg.data}"
            )
            return
        self._local_fly_choice_pub.publish(msg)
        self.get_logger().info(
            f"bridged {self._fly_choice_topic}={msg.data} from "
            f"DOMAIN={self._remote_domain_id} to "
            f"DOMAIN={self._local_domain_id}"
        )

    def _on_drone_state(self, msg: UInt8) -> None:
        state = int(msg.data)
        if state < 1 or state > 5:
            self.get_logger().warning(
                f"ignoring invalid local /drone_state={state}"
            )
            return
        with self._lock:
            changed = self._drone_state != state
            self._drone_state = state
        if changed:
            self._publish_drone_state(state)

    def _pose(self) -> tuple[float | None, float | None]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame, self._robot_frame, Time()
            )
        except TransformException:
            return None, None
        translation = transform.transform.translation
        return (
            round(float(translation.x) * 100.0, 2),
            round(float(translation.y) * 100.0, 2),
        )

    def _publish_drone_state(self, state: int) -> None:
        msg = UInt8()
        msg.data = state
        self._drone_state_pub.publish(msg)

    def _publish_telemetry(self) -> None:
        x_cm, y_cm = self._pose()
        if x_cm is not None and y_cm is not None:
            coordinate = Float32MultiArray()
            coordinate.data = [float(x_cm), float(y_cm)]
            self._coordinate_pub.publish(coordinate)
        with self._lock:
            state = self._drone_state
        if state is not None:
            self._publish_drone_state(state)

    def shutdown(self) -> None:
        try:
            self._remote_executor.shutdown()
        except Exception:
            pass
        self._remote_thread.join(timeout=2.0)
        try:
            self._remote_node.destroy_node()
        except Exception:
            pass
        try:
            self._remote_context.shutdown()
        except Exception:
            pass
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
