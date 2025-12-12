import pickle
import numpy as np
from scipy.spatial import KDTree
from matplotlib import pyplot as plt

import torch
import torch.nn as nn
import torch.nn.functional as F

# ============================================================
#                 MDN 反解 IK 模型
# ============================================================
device = "cuda" if torch.cuda.is_available() else "cpu"
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
        self.hidden = nn.Sequential(
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
    m = mu.size(2)
    target = target.unsqueeze(1).expand_as(mu)
    exponent = -0.5 * torch.sum(((target - mu) / sigma) ** 2, dim=2)
    coef = 1.0 / (torch.prod(sigma, dim=2) * ((2 * np.pi) ** (m / 2)))
    probs = pi * coef * torch.exp(exponent)
    nll = -torch.log(torch.sum(probs, dim=1) + 1e-8)
    return torch.mean(nll)


# ----------------- 机械臂正解 FK（末端 xy） -----------------

def forward_kinematics_from_angles(theta1, theta2, theta3):
    r1 = 0.325
    r2 = 0.275
    r3 = 0.225

    x1 = r1 * np.cos(theta1)
    x2 = x1 + r2 * np.cos(theta1 + theta2)
    x3 = x2 + r3 * np.cos(theta1 + theta2 + theta3)

    y1 = r1 * np.sin(theta1)
    y2 = y1 + r2 * np.sin(theta1 + theta2)
    y3 = y2 + r3 * np.sin(theta1 + theta2 + theta3)

    return np.array([[0, x1, x2, x3],
                     [0, y1, y2, y3]], dtype=np.float32)


def sample_from_mdn_multiple(pi, sigma, mu, num_samples=5000):
    """
    一次性从 MDN 里采样 num_samples 组 (theta1,theta2,theta3)
    """
    batch_size, num_gaussians, out_dim = mu.shape
    pi_np = pi.detach().cpu().numpy()

    indices = np.array([
        np.random.choice(num_gaussians, size=num_samples, p=p)
        for p in pi_np
    ])  # (batch_size, num_samples)

    indices_torch = torch.tensor(indices, device=mu.device)
    batch_indices = torch.arange(batch_size, device=mu.device).unsqueeze(1).expand(-1, num_samples)

    chosen_mu = mu[batch_indices, indices_torch]       # (B, S, out_dim)
    chosen_sigma = sigma[batch_indices, indices_torch] # (B, S, out_dim)

    eps = torch.randn_like(chosen_mu)
    samples = chosen_mu + chosen_sigma * eps

    return samples.detach().cpu().numpy()  # (B, S, out_dim)


# ----------------- MDN IK：求多解，再选一个 -----------------
model = IK_MDN_Model(input_size=2, output_size=3, num_gaussians=50)
model.load_state_dict(torch.load("/home/jimmy/Downloads/MDN_robot/ik_mdn_state.pt"))
model.to("cuda")
model.eval()


def get_ik_solutions(point, num_samples=5000, tol=0.005, max_solutions=20):
    """
    返回一堆满足 FK(x,y)≈point 的 IK 解
    """
    point = np.array(point, dtype=np.float32)
    with torch.no_grad():
        x = torch.tensor([point], dtype=torch.float32, device=device)
        pi, sigma, mu = model(x)
        samples = sample_from_mdn_multiple(pi, sigma, mu, num_samples=num_samples)[0]

    valid = []
    for s in samples:
        fk = forward_kinematics_from_angles(*s)
        x_fk, y_fk = fk[0, -1], fk[1, -1]
        if np.linalg.norm(np.array([x_fk, y_fk]) - point) < tol:
            valid.append([float(round(i, 4)) for i in s])
            if len(valid) >= max_solutions:
                break
    return valid


def pick_ik(point, ref_q=None, **kwargs):
    """
    从 get_ik_solutions 中选一个更“好”的解:
    - 如果 ref_q 给了，就选离 ref_q 最近的
    - 否则选关节范数最小的
    """
    sols = get_ik_solutions(point, **kwargs)
    if not sols:
        return None

    sols = np.array(sols, dtype=np.float32)
    if ref_q is not None:
        ref = np.array(ref_q, dtype=np.float32)
        dists = np.linalg.norm(sols - ref[None, :], axis=1)
    else:
        dists = np.linalg.norm(sols, axis=1)
    best_idx = int(np.argmin(dists))
    return sols[best_idx]


# ============================================================
#                 基于“内侧切点”的绕障规划器
# ============================================================

class TangentPlanner:
    def __init__(self):
        """
        obstacles: list of ('circle', (cx,cy,r)), 只用圆形障碍
        """
        #self.obstacles = obstacles
        self.link_lengths = [0.325, 0.275, 0.225]

    # ---------- FK，给关节角返回末端路径 ----------

    def forward_kinematics(self, q):
        return forward_kinematics_from_angles(q[0], q[1], q[2])

    # ---------- 基本几何：线段 vs 圆 ----------

    @staticmethod
    def line_circle_collision(x1, y1, x2, y2, cx, cy, r):
        d = np.array([x2 - x1, y2 - y1])
        f = np.array([x1 - cx, y1 - cy])
        a = np.dot(d, d)
        b = 2 * np.dot(f, d)
        c = np.dot(f, f) - r * r
        D = b * b - 4 * a * c
        if D < 0:
            return False
        D = np.sqrt(D)
        t1 = (-b - D) / (2 * a)
        t2 = (-b + D) / (2 * a)
        return (0 <= t1 <= 1) or (0 <= t2 <= 1)

    def is_config_colliding(self, q,obstacles):
        xs, ys = self.forward_kinematics(q)
        for x1, y1, x2, y2 in zip(xs, ys, xs[1:], ys[1:]):
            for  params in obstacles: #            for shape, params in obstacles:
                if 1:# shape == 'circle':
                    cx, cy, r = params
                    if self.line_circle_collision(x1, y1, x2, y2, cx, cy, r):
                        return True
        return False

    def segment_collision(self, q1, q2,obstacles, resolution=0.03):
        q1 = np.array(q1, dtype=np.float32)
        q2 = np.array(q2, dtype=np.float32)
        dist = np.linalg.norm(q2 - q1)
        steps = max(2, int(dist / resolution))
        for a in np.linspace(0, 1, steps):
            q = q1 + a * (q2 - q1)
            if self.is_config_colliding(q,obstacles):
                return True
        return False

    # ---------- 找“最近障碍（按中心距原点）” ----------

    def nearest_obstacle_to_origin(self,obstacles):
        best = None
        best_d = float("inf")
        #for shape, params in obstacles:
            #if shape != 'circle':
            #    continue
        for  params in obstacles:
            cx, cy, r = params
            d = np.sqrt(cx * cx + cy * cy)
            if d < best_d:
                best_d = d
                best = (cx, cy, r)
        return best

    @staticmethod
    def inner_tangent_point_to_origin(cx, cy, r, offset=0.02):
        """
        返回靠近原点一侧的切点（再往原点方向收缩一点 offset，保证在内侧）
        """
        d = np.sqrt(cx * cx + cy * cy)
        ux, uy = cx / d, cy / d  # 从原点指向障碍中心的单位向量
        # 圆上靠原点一侧的点 = 中心 - r * 单位向量
        tx = cx - ux * r
        ty = cy - uy * r
        # 再往原点收缩一点，确保在圆内侧一点点
        tx -= ux * offset
        ty -= uy * offset
        return tx, ty

    @staticmethod
    def joint_interp(q1, q2, steps=60):
        q1 = np.array(q1, dtype=np.float32)
        q2 = np.array(q2, dtype=np.float32)
        return [tuple(q1 + a * (q2 - q1)) for a in np.linspace(0, 1, steps)]
    def generate_inner_arc_waypoints(self,  cx, cy, r,
                                     radial_margin=0.03,
                                     angular_span_deg=70,
                                     n_points=7):
        """
        在“障碍靠原点一侧”生成一小段圆弧上的 candidate waypoints。
        - center = (cx, cy), 半径用 r + radial_margin（略微在障碍外侧）
        - 弧线围绕 “中心→原点” 方向，两侧各 angular_span_deg/2
        - 只保留“距离原点 < 障碍中心距离”的点（确保在内侧）
        """
        C = np.array([cx, cy], dtype=np.float32)
        d_c = np.linalg.norm(C)
        if d_c < 1e-6:
            return []

        # 从中心看向原点方向
        base_angle = np.arctan2(-cy, -cx)  # center → origin 的方向角
        phi_max = np.deg2rad(angular_span_deg)
        phis = np.linspace(-phi_max/2, phi_max/2, n_points)

        Rw = r + radial_margin  # 在障碍外面一点点

        waypoints = []
        for phi in phis:
            theta = base_angle + phi
            x = cx + Rw * np.cos(theta)
            y = cy + Rw * np.sin(theta)

            # 只保留“比障碍中心更靠内侧”的点（离原点更近）
            if np.linalg.norm([x, y]) < d_c:
                waypoints.append((float(x), float(y)))

        # 前面我们还有一个“纯内侧切点”（径向方向），一起作为候选
        tx, ty = self.inner_tangent_point_to_origin(cx, cy, r, offset=0.02)
        if np.linalg.norm([tx, ty]) < d_c:
            waypoints.insert(0, (float(tx), float(ty)))

        print(f"生成内侧 waypoint 数量: {len(waypoints)}")
        return waypoints
    def plan(self, q_start, goal_xy,obstacles,
             ik_samples=5000,
             tol=0.005,
             interp_steps=60):

        print("\n===== Tangent Planner Start =====\n")

        # ==========================================
        # 1. 起点与终点 IK 求解
        # ==========================================
        q_start = q_start#pick_ik(start_xy, num_samples=ik_samples, tol=tol)
        q_goal  = pick_ik(goal_xy, num_samples=ik_samples, tol=tol, ref_q=q_start)

        if q_start is None or q_goal is None:
            print("❌ Start 或 Goal 无 IK 解")
            return None

        q_start = np.array(q_start, dtype=np.float32)
        q_goal  = np.array(q_goal, dtype=np.float32)

        print("Start IK:", q_start)
        print("Goal  IK:", q_goal)

        # ==========================================
        # 2. 最近障碍
        # ==========================================
        nearest = self.nearest_obstacle_to_origin(obstacles)
        if nearest is None:
            print("无障碍 → 直接插值")
            return self.joint_interp(q_start, q_goal, interp_steps)

        cx, cy, r = nearest
        print(f"最近障碍 center=({cx:.3f},{cy:.3f}), r={r:.3f}")

        # ==========================================
        # 3. 在障碍靠原点一侧生成一串 waypoint（包含“内侧切点”）
        # ==========================================
        candidate_wps = self.generate_inner_arc_waypoints(cx, cy, r,radial_margin=0.03,angular_span_deg=80, n_points=9)

        if not candidate_wps:
            print("❌ 内侧 arc 上没有有效 waypoint")
            return None

        # ==========================================
        # 4. 遍历所有 waypoint & 其 IK 解，寻找“完全无碰撞”的组合
        # ==========================================
        best_q_tangent = None
        best_wp_xy = None

        for wp_xy in candidate_wps:
            print(f"\n尝试 waypoint XY: {wp_xy}")

            tangent_sols = get_ik_solutions(
                wp_xy,
                num_samples=8000,
                tol=tol,
                max_solutions=40
            )

            if not tangent_sols:
                print("  - 该 waypoint 无 IK 解，跳过")
                continue

            print(f"  - 该 waypoint IK 解数量: {len(tangent_sols)}")

            for sol in tangent_sols:
                sol = np.array(sol, dtype=np.float32)

                # ① start → waypoint 是否碰撞？
                if self.segment_collision(q_start, sol,obstacles):
                    # print("    · start→wp 碰撞")
                    continue

                # ② waypoint → goal 是否碰撞？
                if self.segment_collision(sol, q_goal,obstacles):
                    # print("    · wp→goal 碰撞")
                    continue

                # ✅ 找到了一个完全无碰撞的 IK
                best_q_tangent = sol
                best_wp_xy = wp_xy
                print("    ✔ 找到无碰撞 Tangent IK:", best_q_tangent)
                break

            if best_q_tangent is not None:
                break

        if best_q_tangent is None:
            print("❌ 所有 waypoint 的所有 IK 姿态均发生碰撞 → 无法绕障")
            return None

        print(f"\n最终选用 waypoint XY: {best_wp_xy}")
        print("Tangent IK:", best_q_tangent)

        # ==========================================
        # 5. 生成插值路径：start → tangent → goal
        # ==========================================
        print("生成插值路径 ...")
        path1 = self.joint_interp(q_start, best_q_tangent, interp_steps)
        path2 = self.joint_interp(best_q_tangent, q_goal, interp_steps)

        full_path = path1 + path2[1:]

        print(f"✅ 路径生成完成，共 {len(full_path)} 个关节点")
        print("\n===== Tangent Planner End =====\n")

        return full_path

    # ---------- 核心：规划 Start_xy → 内侧切点 → Goal_xy ----------
    def plan_old2(self, start_xy, goal_xy,
             ik_samples=5000,
             tol=0.005,
             interp_steps=60):

        print("\n===== Tangent Planner Start =====\n")

        # ==========================================
        # 1. 起点与终点 IK 求解
        # ==========================================
        q_start = pick_ik(start_xy, num_samples=ik_samples, tol=tol)
        q_goal  = pick_ik(goal_xy, num_samples=ik_samples, tol=tol, ref_q=q_start)

        if q_start is None or q_goal is None:
            print("❌ Start 或 Goal 无 IK 解")
            return None

        print("Start IK:", q_start)
        print("Goal  IK:", q_goal)

        # ==========================================
        # 2. 获取最近障碍（按中心距原点）
        # ==========================================
        nearest = self.nearest_obstacle_to_origin()
        if nearest is None:
            print("无障碍 → 直接插值")
            return self.joint_interp(q_start, q_goal, interp_steps)

        cx, cy, r = nearest
        print(f"最近障碍 center=({cx:.3f},{cy:.3f}), r={r:.3f}")

        # ==========================================
        # 3. 计算内侧切点（靠原点）
        # ==========================================
        tx, ty = self.inner_tangent_point_to_origin(cx, cy, r, offset=0.02)
        tangent_xy = (tx, ty)
        print(f"内侧切点: ({tx:.3f}, {ty:.3f})")

        # ==========================================
        # 4. 获取多个 tangent IK，并且筛选“安全 IK”
        # ==========================================
        tangent_sols = get_ik_solutions(
            tangent_xy, num_samples=8000, tol=tol, max_solutions=40
        )

        if not tangent_sols:
            print("❌ 切点无 IK 解")
            return None

        # 尝试所有 IK 解，找一个不碰撞的
        q_tangent = None
        print(f"切点 IK 解数量: {len(tangent_sols)}，开始筛选安全 IK ...")

        for sol in tangent_sols:
            sol = np.array(sol, dtype=np.float32)

            # ① start → tangent 是否穿障？
            if self.segment_collision(q_start, sol):
                continue

            # ② tangent → goal 是否穿障？
            if self.segment_collision(sol, q_goal):
                continue

            # 找到一个安全 IK！
            q_tangent = sol
            print("✔ 选用安全的 Tangent IK:", q_tangent)
            break

        if q_tangent is None:
            print("❌ 所有 Tangent IK 姿态均碰撞 → 无法绕障")
            return None

        # ==========================================
        # 5. 插值路径：start→tangent → goal
        # ==========================================
        print("生成插值路径 ...")
        path1 = self.joint_interp(q_start, q_tangent, interp_steps)
        path2 = self.joint_interp(q_tangent, q_goal, interp_steps)

        full_path = path1 + path2[1:]

        print(f"✅ 路径生成完成，共 {len(full_path)} 个关节点")
        print("\n===== Tangent Planner End =====\n")

        return full_path

    def plan_old(self, start_xy, goal_xy,
             ik_samples=5000,
             tol=0.005,
             interp_steps=60):
        """
        返回整条关节空间路径: [q0, q1, ..., qN]
        """

        # 1. IK：起点与终点
        q_start = pick_ik(start_xy, num_samples=ik_samples, tol=tol)
        q_goal = pick_ik(goal_xy, num_samples=ik_samples, tol=tol, ref_q=q_start)

        if q_start is None or q_goal is None:
            print("IK 失败：起点或终点没有解")
            return None

        print("Start IK:", q_start)
        print("Goal  IK:", q_goal)

        # 如果没有障碍，直接直线插值
        nearest = self.nearest_obstacle_to_origin()
        if nearest is None:
            print("无障碍，直接直线插值")
            path = self.joint_interp(q_start, q_goal, interp_steps)
            return path

        cx, cy, r = nearest
        print(f"最近障碍: center=({cx:.3f},{cy:.3f}), r={r:.3f}")

        # 2. 计算内侧切点
        tx, ty = self.inner_tangent_point_to_origin(cx, cy, r, offset=0.02)
        tangent_xy = (tx, ty)
        print(f"内侧切点 (tangent waypoint): ({tx:.3f}, {ty:.3f})")

        # 3. IK：切点
        q_tangent = pick_ik(tangent_xy, num_samples=ik_samples, tol=tol, ref_q=q_start)
        if q_tangent is None:
            print("切点 IK 失败，尝试不经过切点直接插值")
            path_direct = self.joint_interp(q_start, q_goal, interp_steps)
            if self.segment_collision(q_start, q_goal):
                print("直接路径碰撞，失败")
                return None
            return path_direct

        print("Tangent IK:", q_tangent)

        # 4. 插值三段路径：start→tangent → goal
        path1 = self.joint_interp(q_start, q_tangent, interp_steps)
        path2 = self.joint_interp(q_tangent, q_goal, interp_steps)

        full_path = path1 + path2[1:]  # 避免重复 q_tangent

        # 5. 最后做一遍碰撞检查
        print("检查 start → tangent 碰撞...")
        if self.segment_collision(q_start, q_tangent):
            print("start → tangent 路径碰撞！")
        print("检查 tangent → goal 碰撞...")
        if self.segment_collision(q_tangent, q_goal):
            print("tangent → goal 路径碰撞！")

        return full_path


# ============================================================
#                        测试 main
# ============================================================

def test_tangent_planner():
    print("\n========== Tangent Planner TEST ==========\n")

    # 1. 障碍（圆形）
    obstacles = [
        ('circle', (0.40, 0.35, 0.08)),
        # 你可以再加几个：
        # ('circle', (0.22, 0.45, 0.06)),
    ]

    # 2. Start / Goal 的末端 xy
    start_xy = (0.75, 0.05)
    goal_xy = (0.25, 0.35)

    planner = TangentPlanner(obstacles)

    # 3. 规划
    path = planner.plan(start_xy, goal_xy,
                        ik_samples=5000,
                        tol=0.005,
                        interp_steps=50)

    if path is None:
        print("❌ 没有找到路径")
        return

    print(f"\n✅ 找到路径，共 {len(path)} 个关节点")

    # 4. 把末端轨迹画出来
    xs, ys = [], []
    for q in path:
        fk = planner.forward_kinematics(q)
        xs.append(fk[0, -1])
        ys.append(fk[1, -1])

    plt.figure(figsize=(6, 6))
    plt.plot(xs, ys, '-o', label="End Effector Path")

    # 障碍
    ax = plt.gca()
    for shape, params in obstacles:
        if shape == 'circle':
            cx, cy, r = params
            circle = plt.Circle((cx, cy), r, color='red', alpha=0.3)
            ax.add_patch(circle)

    # Start / Goal / Tangent 画一下
    plt.scatter([start_xy[0]], [start_xy[1]], color='green', s=80, label="Start XY")
    plt.scatter([goal_xy[0]], [goal_xy[1]], color='blue', s=80, label="Goal XY")

    plt.title("Tangent Waypoint Path (End Effector)")
    plt.legend()
    plt.axis("equal")
    plt.grid(True)
    plt.show()

    print("\n========== TEST END ==========\n")


def main():
    test_tangent_planner()


if __name__ == "__main__":
    main()
