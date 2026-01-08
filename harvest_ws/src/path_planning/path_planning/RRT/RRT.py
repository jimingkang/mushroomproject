import pickle
import time
import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
from matplotlib.animation import FuncAnimation
from scipy.interpolate import CubicSpline
import numpy as np
from tqdm import tqdm
from scipy.spatial import KDTree
from matplotlib import pyplot as plt, patches

class MDNLayer(nn.Module):
    def __init__(self, in_features, out_features, num_gaussians):
        super().__init__()
        self.num_gaussians = num_gaussians
        self.out_features = out_features

        self.pi = nn.Linear(in_features, num_gaussians)
        self.sigma = nn.Linear(in_features, num_gaussians * out_features)
        self.mu = nn.Linear(in_features, num_gaussians * out_features)

    def forward(self, x):
        pi = F.softmax(self.pi(x), dim=1)
        sigma = F.softplus(self.sigma(x)) + 1e-6
        mu = self.mu(x)

        sigma = sigma.view(-1, self.num_gaussians, self.out_features)
        mu = mu.view(-1, self.num_gaussians, self.out_features)

        return pi, sigma, mu

class IK_MDN_Model(nn.Module):
    def __init__(self, input_size=2, output_size=3, num_gaussians=5, dropout_prob=0.1):
        super().__init__()
        self.hidden =  nn.Sequential(
            nn.Linear(input_size, 128),
            nn.ReLU(),
            nn.Dropout(dropout_prob),
            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Dropout(dropout_prob),
            nn.Linear(128, 128),
            nn.ReLU(),
        )
        self.mdn = MDNLayer(in_features=128, out_features=output_size, num_gaussians=num_gaussians)

    def forward(self, x):
        features = self.hidden(x)
        pi, sigma, mu = self.mdn(features)
        return pi, sigma, mu

def mdn_loss(pi, sigma, mu, target):
    """Computes the negative log-likelihood loss for an MDN."""
    m = mu.size(2)
    target = target.unsqueeze(1).expand_as(mu)
    exponent = -0.5 * torch.sum(((target - mu) / sigma) ** 2, dim=2)
    coef = 1.0 / (torch.prod(sigma, dim=2) * ((2 * np.pi) ** (m / 2)))
    probs = pi * coef * torch.exp(exponent)
    nll = -torch.log(torch.sum(probs, dim=1) + 1e-8)
    return torch.mean(nll)



def forward_kinematics_from_angles(theta1,theta2,theta3):
    r1= 0.325
    r2 = 0.275
    r3=0.265  #0.225
    x1 = r1 * np.cos(theta1)
    x2= x1 + r2 * np.cos(theta1 + theta2)
    x3 = x2 + r3 * np.cos(theta1 + theta2 + theta3)

    y1 = r1 * np.sin(theta1)
    y2 = y1 + r2 * np.sin(theta1 + theta2)
    y3 = y2 + r3 * np.sin(theta1 + theta2 + theta3)


    return [[0,0,],[x1,y1],[x2,y2],[x3,y3]]

def sample_from_mdn_multiple(pi, sigma, mu, num_samples=5_000):
    """Vectorized: sample 5000 times from MDN output (no Python loop)."""
    batch_size, num_gaussians, out_dim = mu.shape

    # (batch_size, num_gaussians) -> numpy for multinomial sampling
    pi_np = pi.detach().cpu().numpy()

    # Sample 5000 component indices for each batch according to pi
    indices = np.array([
        np.random.choice(num_gaussians, size=num_samples, p=p)
        for p in pi_np
    ])  # shape: (batch_size, num_samples)

    indices_torch = torch.tensor(indices, device=mu.device)

    # Gather corresponding mu and sigma for all samples
    batch_indices = torch.arange(batch_size).unsqueeze(1).expand(-1, num_samples)
    chosen_mu = mu[batch_indices, indices_torch]      # (batch_size, num_samples, out_dim)
    chosen_sigma = sigma[batch_indices, indices_torch]# (batch_size, num_samples, out_dim)

    # Sample from Normal(mu, sigma)
    eps = torch.randn_like(chosen_mu)
    samples = chosen_mu + chosen_sigma * eps          # (batch_size, num_samples, out_dim)

    return samples.detach().cpu().numpy()

def get_ik(point):
    input_x = torch.tensor([point], dtype=torch.float32,device="cuda")
    pi, sigma, mu = model(input_x)
    samples = sample_from_mdn_multiple(pi, sigma, mu)[0]
    valid_sample=[]
    for sample in samples:
        x,y=forward_kinematics_from_angles(*sample)[-1]
        if np.linalg.norm(np.array([x,y])-np.array(point))<0.005:
            valid_sample.append([round(i,4) for i in sample])
    return valid_sample




class RRTAngle:
    def __init__(self, start_pos, obstacles=[], tolerance=0.1, step_size=0.2):
        self.length = [0.325, 0.275, 0.225]
        self.workspace_size = sum(self.length)
        self.start_node = np.array(start_pos, dtype=np.float32)
        self.tolerance = tolerance
        self.step_size = step_size
        self.tree = {tuple(self.start_node): None}
        self.goal_node = None
        self.obstacles = obstacles
        self.forward_kinematics_cache = {}
        self.kd_tree = KDTree([self.start_node])
        # ✅ 碰撞反馈的代价地图：存储危险区域（工作空间里）
        self.cost_map = []   # 每个元素: {"center": np.array([x,y]), "radius": r, "weight": w}

        self.pre_paths=[]
        

    def add_obstacle(self, shape, params):
        self.obstacles.append((shape, params))

    def forward_kinematics(self, angles):
        angles_tuple = tuple([round(float(i), 4) for i in angles])
        if angles_tuple in self.forward_kinematics_cache:
            return self.forward_kinematics_cache[angles_tuple]

        x0, y0 = 0, 0
        x1 = x0 + self.length[0] * np.cos(angles[0])
        y1 = y0 + self.length[0] * np.sin(angles[0])
        x2 = x1 + self.length[1] * np.cos(angles[0] + angles[1])
        y2 = y1 + self.length[1] * np.sin(angles[0] + angles[1])
        x3 = x2 + self.length[2] * np.cos(angles[0] + angles[1] + angles[2])
        y3 = y2 + self.length[2] * np.sin(angles[0] + angles[1] + angles[2])

        result = np.array([[x0, x1, x2, x3], [y0, y1, y2, y3]], dtype=np.float32)
        self.forward_kinematics_cache[angles_tuple] = result
        return result

    # ✅ 点到线段距离，用于碰撞反馈中测最近距离
    def _distance_point_to_segment(self, px, py, x1, y1, x2, y2):
        A = np.array([x1, y1], dtype=np.float32)
        B = np.array([x2, y2], dtype=np.float32)
        P = np.array([px, py], dtype=np.float32)

        AB = B - A
        denom = np.dot(AB, AB) + 1e-9
        t = np.dot(P - A, AB) / denom
        t = np.clip(t, 0.0, 1.0)
        closest = A + t * AB
        return float(np.linalg.norm(P - closest))

    # ✅ 从碰撞中提取反馈：哪个链段、离障碍多近、障碍中心在哪
    def extract_collision_feedback(self, q_near, q_new):
        xs, ys = self.forward_kinematics(q_new)  # xs, ys: 4 个关节/末端坐标
        feedback = {
            "min_distance": float("inf"),
            "link_index": None,
            "center": None,
        }

        for link_id, (x1, y1, x2, y2) in enumerate(zip(xs, ys, xs[1:], ys[1:])):
            for shape, params in self.obstacles:
                if shape == "circle":
                    cx, cy, r = params
                    dist = self._distance_point_to_segment(cx, cy, x1, y1, x2, y2)
                    center = np.array([cx, cy], dtype=np.float32)
                elif shape == "rectangle":
                    rx, ry, w, h = params
                    cx = rx + w / 2.0
                    cy = ry + h / 2.0
                    dist = self._distance_point_to_segment(cx, cy, x1, y1, x2, y2)
                    center = np.array([cx, cy], dtype=np.float32)
                else:
                    continue

                if dist < feedback["min_distance"]:
                    feedback["min_distance"] = dist
                    feedback["link_index"] = link_id
                    feedback["center"] = center

        return feedback

    # ✅ 更新碰撞代价地图：把“危险区域”记录下来
    def update_cost_map(self, feedback):
        if feedback["center"] is None:
            return
        d = feedback["min_distance"]
        # 距离越小，权重越大；这里做一个简单的映射
        radius = max(0.10, 0.20 - 0.5 * d)   # 在障碍附近形成一个 ~0.1–0.2 m 的避让半径
        weight = max(1.0, 2.0 - d * 5.0)
        entry = {
            "center": feedback["center"],
            "radius": radius,
            "weight": weight,
        }
        self.cost_map.append(entry)

    # ✅ 采样时根据 cost_map 做简单的“拒绝采样”，让末端远离高代价区域
    def bias_sample_away_from_cost(self, sample, max_retry=5):
        sample = np.array(sample, dtype=np.float32)

        for _ in range(max_retry):
            xs, ys = self.forward_kinematics(sample)
            ee = np.array([xs[-1], ys[-1]], dtype=np.float32)

            bad = False
            for entry in self.cost_map:
                center = entry["center"]
                radius = entry["radius"]
                if np.linalg.norm(ee - center) < radius:
                    bad = True
                    break

            if not bad:
                return sample

            # 如果当前位置在高代价区域内，则重新随机采样一组关节角
            sample = np.array([
                np.random.uniform(-np.pi / 2, np.pi / 2),
                np.random.uniform(-3 * np.pi / 4, 3 * np.pi / 4),
                np.random.uniform(-3 * np.pi / 4, 3 * np.pi / 4),
            ], dtype=np.float32)

        return sample

    def sample_position_new(self, ik_bais, goal_bias=0.2):
        if ik_bais is not None and len(ik_bais) > 0 and np.random.rand() < goal_bias:
            index = np.random.choice([i for i in range(0, len(ik_bais))])
            sample = np.array(ik_bais[index], dtype=np.float32)
        else:
            sample = np.array([
                np.random.uniform(-np.pi / 2, np.pi / 2),
                np.random.uniform(-3 * np.pi / 4, 3 * np.pi / 4),
                np.random.uniform(-3 * np.pi / 4, 3 * np.pi / 4),
            ], dtype=np.float32)

        # ✅ 在这里加入碰撞反馈驱动的采样偏置
        sample = self.bias_sample_away_from_cost(sample)
        return sample
    def sample_position(self, goal_ws=None, goal_bias=0.2):
        if goal_ws is not None and np.random.rand() < goal_bias:
            # Try goal-biased sampling
            for _ in range(10):  # Try multiple times to get a good sample
                sample = [np.random.uniform(-np.pi/2, np.pi/2),
                        np.random.uniform(-3*np.pi/4, 3*np.pi/4),
                        np.random.uniform(-3*np.pi/4, 3*np.pi/4)]
                xs, ys = self.forward_kinematics(sample)
                end_effector = np.array([xs[-1], ys[-1]])
                if np.linalg.norm(end_effector - goal_ws) < self.workspace_size * 0.2:
                    return sample
        # Otherwise, uniform sampling
        return [np.random.uniform(-np.pi/2, np.pi/2),
                np.random.uniform(-3*np.pi/4, 3*np.pi/4),
                np.random.uniform(-3*np.pi/4, 3*np.pi/4)]

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

    def nearest_node(self, sample):
        dist, idx = self.kd_tree.query([sample], k=1)
        nearest_node = tuple(self.kd_tree.data[idx][0])
        return nearest_node

    def steer(self, from_node, to_node):
        direction = np.array(to_node, dtype=np.float32) - np.array(from_node, dtype=np.float32)
        distance = np.linalg.norm(direction)
        if distance <= self.step_size:
            return np.array(to_node, dtype=np.float32)
        direction /= distance
        return np.array(from_node, dtype=np.float32) + self.step_size * direction

    def distance_in_workspace(self, angles, goal_ws):
        xs, ys = self.forward_kinematics(angles)
        end_effector = np.array([xs[-1], ys[-1]])
        distance = np.linalg.norm(end_effector - goal_ws)
        return distance

    def build_tree(self, goal_ws, max_iterations=500, goal_bias=0.2):
        goal_ws = np.array(goal_ws, dtype=np.float32)
        #ik_bais = get_ik(goal_ws)
        #print("vaild ik sol:", len(ik_bais))
        for i in range(max_iterations):
            #sample = self.sample_position(ik_bais, goal_bias if len(ik_bais) > 0 else -1)
            sample = self.sample_position(goal_ws)
            nearest = self.nearest_node(sample)
            new_node = self.steer(nearest, sample)
            new_node_tuple = tuple(new_node)

            distance = self.distance_in_workspace(new_node, goal_ws)

            if new_node_tuple in self.tree:
                continue

            # ✅ 碰撞反馈 RRT：这里不只是 continue，而是记录碰撞信息
            if self.is_in_obstacle(new_node):
                feedback = self.extract_collision_feedback(nearest, new_node)
                self.update_cost_map(feedback)
                continue

            self.tree[new_node_tuple] = tuple(nearest)
            self.kd_tree = KDTree(list(self.tree.keys()))  # Rebuild the KDTree after adding new node

            if distance < self.tolerance:
                self.goal_node = new_node_tuple
                path = self.reconstruct_path()
                return path

        return None

    def reconstruct_path(self):
        path = []
        node = self.goal_node
        while node is not None:
            path.append(node)
            node = self.tree[node]
        return path[::-1]

    # 后面 animate / edge_is_collision_free / smooth_path / smooth_curve 原样保留
    # 你原来的这些函数可以不改，直接接在这里即可

    def animate(self, path, goal_ws=None, interval=100):
        """
        动画展示 SCARA 机械臂的运动轨迹
        :param path: RRT 生成的角度列表 [[theta1, theta2, theta3], ...]
        :param goal_ws: 目标点坐标 [x, y]
        :param interval: 动画帧间隔(ms)
        """
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.set_aspect('equal')

        # 设置显示范围 (稍微比机械臂最大长度大一点)
        limit = self.workspace_size * 1.1
        ax.set_xlim(-limit, limit)
        ax.set_ylim(-limit, limit)
        ax.grid(True, linestyle='--', alpha=0.5)
        ax.set_title(f"SCARA RRT Path (Steps: {len(path)})")

        # 1. 绘制障碍物
        for shape, params in self.obstacles:
            if shape == 'circle':
                cx, cy, r = params
                circle = patches.Circle((cx, cy), r, color='gray', alpha=0.5)
                ax.add_patch(circle)
            elif shape == 'rectangle':
                x, y, w, h = params
                rect = patches.Rectangle((x, y), w, h, color='gray', alpha=0.5)
                ax.add_patch(rect)

        # 2. 绘制起点和终点
        start_coords = self.forward_kinematics(path[0])
        ax.plot(start_coords[0][-1], start_coords[1][-1], 'go', label='Start')  # 起点绿色

        if goal_ws is not None:
            ax.plot(goal_ws[0], goal_ws[1], 'rx', markersize=10, markeredgewidth=2, label='Goal')  # 目标红色X

        ax.legend(loc='upper right')

        # 3. 初始化动态元素
        # 机械臂连杆 (蓝色实线，带节点)
        link_line, = ax.plot([], [], 'o-', linewidth=4, markersize=8, color='cornflowerblue')
        # 末端轨迹 (红色虚线)
        trace_line, = ax.plot([], [], '--', linewidth=1, color='red', alpha=0.8)

        trace_x, trace_y = [], []

        def init():
            link_line.set_data([], [])
            trace_line.set_data([], [])
            return link_line, trace_line

        def update(frame):
            angles = path[frame]
            # 获取关节坐标 [[x0, x1, x2, x3], [y0, y1, y2, y3]]
            coords = self.forward_kinematics(angles)
            xs = coords[0]
            ys = coords[1]

            # 更新机械臂形态
            link_line.set_data(xs, ys)

            # 更新末端轨迹
            end_effector_x = xs[-1]
            end_effector_y = ys[-1]
            trace_x.append(end_effector_x)
            trace_y.append(end_effector_y)
            trace_line.set_data(trace_x, trace_y)

            return link_line, trace_line

        # 创建动画
        ani = FuncAnimation(fig, update, frames=len(path), init_func=init,
                            interval=interval, blit=True, repeat=True)

        plt.show()
    def edge_is_collision_free(self, q1, q2, step=None):
        q1 = np.asarray(q1, dtype=np.float32)
        q2 = np.asarray(q2, dtype=np.float32)
        dist = np.linalg.norm(q2 - q1)
        if dist == 0:
            return not self.is_in_obstacle(q1)

        if step is None:
            step = self.step_size * 0.5
        n = int(dist / step) + 1
        for a in np.linspace(0, 1, n):
            q = q1 + a * (q2 - q1)
            if self.is_in_obstacle(q):
                return False
        return True
    def smooth_path(self, path, iterations=100):
        """
        基于 Shortcut smoothing 的路径平滑
        :param path: 原始路径 [q0, q1, ..., qN]
        :param iterations: 尝试优化次数
        :return: 平滑后的路径
        """
        if path is None or len(path) < 3:
            return path  # 无需平滑

        path = [np.array(q, dtype=np.float32) for q in path]

        for _ in range(iterations):
            if len(path) <= 2:
                break

            # 随机选择两个节点 i < j
            i = np.random.randint(0, len(path) - 2)
            j = np.random.randint(i + 2, len(path))

            q1 = path[i]
            q2 = path[j]

            # 如果中间可直连 → 删除多余点
            if self.edge_is_collision_free(q1, q2):
                # 保留 i 和 j，中间全部替换成一条直线插值
                new_segment = [q1]

                steps = int(np.linalg.norm(q2 - q1) / self.step_size) + 2
                for a in np.linspace(0, 1, steps):
                    q = q1 + a * (q2 - q1)
                    new_segment.append(q)

                path = path[:i] + new_segment + path[j + 1:]

        return path


    def smooth_curve(self, path, points=200):
        """
        使用三次样条使路径更平滑（适合机械臂控制）
        :param path: 已 shortcut 的路径
        :param points: 生成多少个平滑点
        """
        path = np.array(path)
        N = len(path)
        t = np.linspace(0, 1, N)

        # 每个关节一根 spline
        q1_spline = CubicSpline(t, path[:, 0])
        q2_spline = CubicSpline(t, path[:, 1])
        q3_spline = CubicSpline(t, path[:, 2])

        ts = np.linspace(0, 1, points)

        smooth_path = np.stack([
            q1_spline(ts),
            q2_spline(ts),
            q3_spline(ts)
        ], axis=1)

        return smooth_path

model = IK_MDN_Model(input_size=2, output_size=3, num_gaussians=50)
model.load_state_dict(torch.load("/home/jimmy/Downloads/MDN_robot/ik_mdn_state.pt"))
model.to("cuda")
model.eval()
    