"""ROS command/status adapter. Only the session clock receives user commands."""

import json
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy


class CommandInterface:
    def __init__(self, node, session):
        self.node, self.session = node, session
        self.reply = {}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = node.create_publisher(String, "/biped/controller/status", qos)
        self.subscription = node.create_subscription(
            String, "/biped/controller/command", self.receive, 10
        )
        self.timer = node.create_timer(0.5, self.publish)

    def receive(self, message):
        request = {}
        try:
            request = json.loads(message.data)
            if not isinstance(request, dict):
                raise ValueError("Command must be a JSON object")
            identifier = request.get("id")
            if (
                not isinstance(identifier, str)
                or not identifier
                or len(identifier) > 100
            ):
                raise ValueError("A nonempty request id is required")
            # A repeated delivery must not issue a second walking command.
            if identifier == self.reply.get("request_id"):
                self.publish()
                return
            result = self.session.command(
                request.get("command"), request.get("steps", 1), self.node.ready
            )
            self.reply = dict(request_id=identifier, accepted=True, message=result)
        except (ValueError, TypeError) as error:
            self.reply = dict(
                request_id=request.get("id") if isinstance(request, dict) else None,
                accepted=False,
                message=str(error),
            )
        self.publish()

    def publish(self):
        self.publisher.publish(
            String(
                data=json.dumps(
                    dict(self.session.status(self.node.ready), **self.reply)
                )
            )
        )
