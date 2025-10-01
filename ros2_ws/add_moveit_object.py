import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive


class PlanningSceneInterface(Node):
    def __init__(self, node_name="planning_scene_interface"):
        super().__init__(node_name)
        self._pub_co = self.create_publisher(CollisionObject, "/collision_object", 10)
        self._known_objects = set()

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

        self._pub_co.publish(co)
        self._known_objects.add(name)
        self.get_logger().info(f"Added box {name}")

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

        self._pub_co.publish(co)
        self._known_objects.add(name)
        self.get_logger().info(f"Added cylinder {name}")

    def remove_object(self, name: str):
        co = CollisionObject()
        co.id = name
        co.header.frame_id = "base_link"
        co.operation = CollisionObject.REMOVE

        self._pub_co.publish(co)
        self._known_objects.discard(name)
        self.get_logger().info(f"Removed object {name}")

    def get_known_object_names(self):
        return list(self._known_objects)


# ✅ main 方法
def main():
    rclpy.init()
    scene = PlanningSceneInterface()

    # 定义一个障碍物姿态
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.pose.position.x = 0.5
    pose.pose.position.y = 0.0
    pose.pose.position.z = 0.25
    pose.pose.orientation.w = 1.0

    # 添加一个box
    scene.add_box("box1", pose, (0.2, 0.2, 0.2))

    # 添加一个cylinder
    scene.add_cylinder("cylinder1", pose, height=0.4, radius=0.1)

    print("obj name:", scene.get_known_object_names())
    rclpy.spin(scene)
    scene.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

