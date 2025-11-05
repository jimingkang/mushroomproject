import numpy as np
import random
from tqdm import tqdm
from scipy.spatial import KDTree
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle

L1, L2, L3 = 0.325, 0.275, 0.26


class RRTAngle:
    def __init__(self, start_pos, obstacles=None, tolerance=0.1, step_size=0.2):
        self.length = [L1, L2, L3]
        self.workspace_size = sum(self.length)
        self.start_node = np.array(start_pos)
        self.tolerance = tolerance
        self.step_size = step_size
        self.tree = {tuple(self.start_node): None}
        self.goal_node = None
        self.obstacles = obstacles if obstacles else []
        self.forward_kinematics_cache = {}
        self.kd_tree = KDTree([self.start_node])

    # ================================
    # 基础函数
    # ================================
    def set_start_pos(self, params):
        self.start_node = np.array(params)
        self.tree = {tuple(self.start_node): None}
        self.kd_tree = KDTree([self.start_node])

    def add_obstacle(self, shape, params):
        self.obstacles.append((shape, params))

    def forward_kinematics(self, angles):
        angles_tuple = tuple(angles)
        if angles_tuple in self.forward_kinematics_cache:
            return self.forward_kinematics_cache[angles_tuple]

        x0, y0 = 0, 0
        x1 = x0 + self.length[0] * np.cos(angles[0])
        y1 = y0 + self.length[0] * np.sin(angles[0])
        x2 = x1 + self.length[1] * np.cos(angles[0] + angles[1])
        y2 = y1 + self.length[1] * np.sin(angles[0] + angles[1])
        # ✅ 修复：第三段长度应为 self.length[2]
        x3 = x2 + self.length[2] * np.cos(angles[0] + angles[1] + angles[2])
        y3 = y2 + self.length[2] * np.sin(angles[0] + angles[1] + angles[2])

        result = np.array([[x0, x1, x2, x3], [y0, y1, y2, y3]], dtype=np.float32)
        self.forward_kinematics_cache[angles_tuple] = result
        return result

    # ================================
    # 采样函数
    # ================================
    def sample_position(self, goal_ws=None, goal_bias=0.2):
        if goal_ws is not None and np.random.rand() < goal_bias:
            for _ in range(10):
                sample = [np.random.uniform(-np.pi / 2, np.pi / 2),
                          np.random.uniform(-3 * np.pi / 4, 3 * np.pi / 4),
                          np.random.uniform(-3 * np.pi / 4, 3 * np.pi / 4)]
                xs, ys = self.forward_kinematics(sample)
                end_effector = np.array([xs[-1], ys[-1]])
                if np.linalg.norm(end_effector - goal_ws) < self.workspace_size * 0.2:
                    return sample
        return [np.random.uniform(-np.pi / 2, np.pi / 2),
                np.random.uniform(-3 * np.pi / 4, 3 * np.pi / 4),
                np.random.uniform(-3 * np.pi / 4, 3 * np.pi / 4)]

    # ================================
    # 障碍检测
    # ================================
    def is_in_obstacle(self, angles):
        xs, ys = self.forward_kinematics(angles)
        for x1, y1, x2, y2 in zip(xs, ys, xs[1:], ys[1:]):
            for shape, params in self.obstacles:
                if shape == 'circle':
                    cx, cy, r = params
                    if self._line_circle_collision(x1, y1, x2, y2, cx, cy, r):
                        return True
                elif shape == 'rectangle':
                    x, y, w, h = params
                    if self._line_rect_collision(x1, y1, x2, y2, x, y, w, h):
                        return True
        return False

    def _line_circle_collision(self, x1, y1, x2, y2, cx, cy, r):
        d = np.array([x2 - x1, y2 - y1])
        f = np.array([x1 - cx, y1 - cy])
        a = np.dot(d, d)
        b = 2 * np.dot(f, d)
        c = np.dot(f, f) - r * r
        discriminant = b * b - 4 * a * c
        if discriminant < 0:
            return False
        discriminant = np.sqrt(discriminant)
        t1 = (-b - discriminant) / (2 * a)
        t2 = (-b + discriminant) / (2 * a)
        return (0 <= t1 <= 1) or (0 <= t2 <= 1)

    def _line_rect_collision(self, x1, y1, x2, y2, rx, ry, rw, rh):
        for t in np.linspace(0, 1, 20):
            xt = x1 + t * (x2 - x1)
            yt = y1 + t * (y2 - y1)
            if rx <= xt <= rx + rw and ry <= yt <= ry + rh:
                return True
        return False

    # ================================
    # 距离、延伸、树操作
    # ================================
    def nearest_node(self, sample):
        dist, idx = self.kd_tree.query([sample], k=1)
        nearest_node = tuple(self.kd_tree.data[idx][0])
        return nearest_node

    def steer(self, from_node, to_node):
        direction = np.array(to_node) - np.array(from_node)
        distance = np.linalg.norm(direction)
        if distance <= self.step_size:
            return np.array(to_node)
        direction /= distance
        return np.array(from_node) + self.step_size * direction

    def distance_in_workspace(self, angles, goal_ws):
        xs, ys = self.forward_kinematics(angles)
        end_effector = np.array([xs[-1], ys[-1]])
        return np.linalg.norm(end_effector - goal_ws)

    # ================================
    # RRT主函数
    # ================================
    def build_tree(self, goal_ws, max_iterations=3000):
        goal_ws = np.array(goal_ws, dtype=np.float32)
        for i in tqdm(range(max_iterations)):
            sample = self.sample_position(goal_ws=goal_ws)
            nearest = self.nearest_node(sample)
            new_node = self.steer(nearest, sample)
            new_node_tuple = tuple(new_node)
            if new_node_tuple in self.tree:
                continue
            if self.is_in_obstacle(new_node):
                continue
            distance = self.distance_in_workspace(new_node, goal_ws)
            self.tree[new_node_tuple] = tuple(nearest)
            self.kd_tree = KDTree(list(self.tree.keys()))
            if distance < self.tolerance:
                self.goal_node = new_node_tuple
                print(f"✅ Goal reached in {i} iterations!")
                smooth_curve, _ = self.reconstruct_path()
                return smooth_curve, _
        print("⚠️ Goal not reached within iteration limit.")
        return None, None


    # ================================
    # 路径重建
    # ================================
    def reconstruct_path(self):
        path = []
        node = self.goal_node
        while node is not None:
            path.append(node)
            node = self.tree[node]
        path = path[::-1]

        # 转换到末端坐标轨迹
        smooth_curve = []
        for q in path:
            xs, ys = self.forward_kinematics(q)
            smooth_curve.append([xs[-1], ys[-1]])
        return smooth_curve, path

    # ================================
    # 可视化
    # ================================
    def visualize(self, smooth_curve=None, show_tree=True):
        fig, ax = plt.subplots(figsize=(6, 6))
        ax.set_xlim(-self.workspace_size, self.workspace_size)
        ax.set_ylim(-self.workspace_size, self.workspace_size)
        ax.set_aspect('equal')
        ax.set_title("RRT Path Planning Visualization")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")

        # 绘制障碍物
        for shape, params in self.obstacles:
            if shape == 'circle':
                cx, cy, r = params
                circle = Circle((cx, cy), r, color='gray', alpha=0.5)
                ax.add_patch(circle)
            elif shape == 'rectangle':
                x, y, w, h = params
                rect = Rectangle((x, y), w, h, color='gray', alpha=0.5)
                ax.add_patch(rect)

        # 绘制搜索树
        if show_tree:
            for node, parent in self.tree.items():
                if parent is not None:
                    xs1, ys1 = self.forward_kinematics(node)
                    xs2, ys2 = self.forward_kinematics(parent)
                    ax.plot([xs1[-1], xs2[-1]], [ys1[-1], ys2[-1]], color='lightblue', linewidth=0.5)

        # 绘制最终路径
        if smooth_curve is not None:
            xs, ys = zip(*smooth_curve)
            ax.plot(xs, ys, color='red', linewidth=2.5, label='Final Path')

        # 起点与终点
        start_xs, start_ys = self.forward_kinematics(self.start_node)
        ax.scatter(start_xs[-1], start_ys[-1], color='green', s=100, label='Start')
        if self.goal_node:
            goal_xs, goal_ys = self.forward_kinematics(self.goal_node)
            ax.scatter(goal_xs[-1], goal_ys[-1], color='orange', s=100, label='Goal')

            # 绘制终点机械臂姿态
            ax.plot(goal_xs, goal_ys, '-o', color='black', markersize=4, label='Final Arm Pose')

        ax.legend()
        plt.show()


# ================================
# 测试用例
# ================================
if __name__ == "__main__":
    np.random.seed(0)
    planner = RRTAngle(start_pos=[0.0, 0.0, 0.0])
    planner.add_obstacle('circle', (0.5, 0.5, 0.15))
    planner.add_obstacle('rectangle', (-0.3, 0.4, 0.2, 0.2))

    goal_ws = (0.0, 0.6)
    smooth_curve, _ = planner.build_tree(goal_ws)
    planner.visualize(smooth_curve)
