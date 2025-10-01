import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive


class PlanningSceneInterface(Node):
    def __init__(self, node_name="planning_scene_interface"):
        super().__init__(node_name)
        # 发布 PlanningScene，而不是单独的 CollisionObject
        self._pub_ps = self.create_publisher(PlanningScene, "/monitored_planning_scene", 10)
        self._known_objects = {}

    def _publish_scene(self):
        """把所有障碍物统一发布到 /planning_scene"""
        ps = PlanningScene()
        ps.is_diff = True
        ps.world.collision_objects = list(self._known_objects.values())
        self._pub_ps.publish(ps)
        self.get_logger().info(f"Planning scene updated. Objects: {list(self._known_objects.keys())}")

    def add_box(self, name: str, pose: PoseStamped, size=(0.1, 0.1, 0.1)):
        co = CollisionObject()
        co.id = name
        co.header = pose.header
        co.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(size)

        co.primitives.append(box)
        co.primitive_poses.append(pose.pose)

        self._known_objects[name] = co
        self._publish_scene()

    def add_cylinder(self, name: str, pose: PoseStamped, height: float, radius: float):
        co = CollisionObject()
        co.id = name
        co.header = pose.header
        co.operation = CollisionObject.ADD

        cyl = SolidPrimitive()
        cyl.type = SolidPrimitive.CYLINDER
        cyl.dimensions = [height, radius]

        co.primitives.append(cyl)
        co.primitive_poses.append(pose.pose)

        self._known_objects[name] = co
        self._publish_scene()

    def remove_object(self, name: str):
        if name in self._known_objects:
            co = self._known_objects[name]
            co.operation = CollisionObject.REMOVE
            self._known_objects.pop(name)
            self._publish_scene()
            self.get_logger().info(f"Removed object {name}")

    def get_known_object_names(self):
        return list(self._known_objects.keys())


def main():
    rclpy.init()
    scene = PlanningSceneInterface()

    pose = PoseStamped()
    pose.header.frame_id = "world"   # ⚠️ 用你的 MoveIt TF 里的根坐标系，比如 base_link/world
    pose.pose.position.x = 0.3
    pose.pose.position.y = 0.0
    pose.pose.position.z = 0.0
    pose.pose.orientation.w = 1.0

    #scene.add_box("box1", pose, (0.2, 0.2, 0.2))
    scene.add_cylinder("cylinder1", pose, height=0.8, radius=0.01)

    print(":", scene.get_known_object_names())
    rclpy.spin(scene)
    scene.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

