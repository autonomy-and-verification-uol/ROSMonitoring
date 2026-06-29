"""TL/Reelay property for the turtlesim ROSMonitoring case study.

The oracle observes velocity commands, turtle poses, stamped pose bridge
messages, and teleport requests. The property remains currently true while
filtering-relevant commands stay inside a small safety envelope:

- velocity commands must stay below configured linear/angular limits;
- teleport targets must stay inside the turtlesim window margin.

Pose topics are logged and ordered for observability, but they do not drive the
filtering verdict. This keeps the command and service examples independent: an
old pose near the window edge cannot poison later service requests.
"""

PROPERTY = "{safe}"

MIN_POSITION = 1.0
MAX_POSITION = 10.0
MAX_LINEAR_X = 2.0
MAX_ANGULAR_Z = 2.8

predicates = {
    "time": 0,
    "safe": True,
}


def _number(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _inside_window(x_value, y_value):
    x = _number(x_value)
    y = _number(y_value)
    return MIN_POSITION <= x <= MAX_POSITION and MIN_POSITION <= y <= MAX_POSITION


def _safe_velocity(message):
    linear = message.get("linear") or {}
    angular = message.get("angular") or {}
    return abs(_number(linear.get("x"))) <= MAX_LINEAR_X and abs(_number(angular.get("z"))) <= MAX_ANGULAR_Z


def _safe_teleport(message):
    request = message.get("request") or {}
    return _inside_window(request.get("x"), request.get("y"))


def abstract_message(message):
    topic = message.get("topic")
    service = message.get("service")
    safe = True

    if topic == "turtle1/cmd_vel":
        safe = _safe_velocity(message)
    elif service == "turtle1/teleport_absolute" and "request" in message:
        safe = _safe_teleport(message)

    predicates["safe"] = bool(safe)
    predicates["time"] = message.get("time", predicates["time"])
    return predicates
