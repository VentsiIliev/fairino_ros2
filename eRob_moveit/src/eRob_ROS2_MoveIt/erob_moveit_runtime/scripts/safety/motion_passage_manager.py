"""Generic removable collision lids for guarded Cartesian passages."""

from __future__ import annotations

from threading import RLock

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive


class MotionPassageManager:
    """Publish collision lids that are closed by default and opened explicitly."""

    def __init__(self, node, publisher, passages) -> None:
        self._node = node
        self._publisher = publisher
        self._lock = RLock()
        self._passages = {}
        self._closed = {}
        for raw in passages or []:
            passage = dict(raw or {})
            passage_id = str(passage.get("id", "")).strip()
            size = list(passage.get("box_size_m", []) or [])
            origin = list(passage.get("origin_m", []) or [])
            if not passage_id or len(size) != 3 or len(origin) != 3:
                node.get_logger().error(
                    f"[MotionPassage] Ignoring invalid passage definition: {raw}"
                )
                continue
            self._passages[passage_id] = {
                "id": passage_id,
                "object_id": str(passage.get("object_id", f"motion_passage_lid_{passage_id}")),
                "frame": str(passage.get("frame", "mounting_surface")),
                "box_size_m": [float(value) for value in size],
                "origin_m": [float(value) for value in origin],
            }
            self._closed[passage_id] = True

    def publish_all_closed(self) -> None:
        for passage_id in tuple(self._passages):
            self.set_closed(passage_id, True)

    def set_closed(self, passage_id: str, closed: bool) -> dict:
        passage_id = str(passage_id or "").strip()
        with self._lock:
            passage = self._passages.get(passage_id)
            if passage is None:
                return {"success": False, "error": f"unknown motion passage '{passage_id}'"}
            self._publish(passage, bool(closed))
            self._closed[passage_id] = bool(closed)
            return self.status(passage_id)

    def status(self, passage_id: str | None = None) -> dict:
        with self._lock:
            if passage_id is not None:
                passage_id = str(passage_id).strip()
                if passage_id not in self._passages:
                    return {"success": False, "error": f"unknown motion passage '{passage_id}'"}
                return {
                    "success": True,
                    "passage_id": passage_id,
                    "closed": self._closed[passage_id],
                    "object_id": self._passages[passage_id]["object_id"],
                }
            return {
                "success": True,
                "passages": {
                    key: {"closed": self._closed[key], "object_id": value["object_id"]}
                    for key, value in self._passages.items()
                },
            }

    def _publish(self, passage: dict, closed: bool) -> None:
        obj = CollisionObject()
        obj.id = passage["object_id"]
        obj.header.frame_id = passage["frame"]
        obj.header.stamp = self._node.get_clock().now().to_msg()
        if closed:
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = list(passage["box_size_m"])
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = passage["origin_m"]
            pose.orientation.w = 1.0
            obj.primitives.append(primitive)
            obj.primitive_poses.append(pose)
            obj.operation = CollisionObject.ADD
        else:
            obj.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)
        self._publisher.publish(scene)
        lid_state = "closed" if closed else "open"
        self._node.get_logger().info(
            f"[MotionPassage] passage={passage['id']} lid={lid_state} object_id={obj.id}"
        )
