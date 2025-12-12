import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation, patches
import time

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib import animation
import pickle

import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np



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
    r3=0.265 #0.225
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
def get_ik(point, num_samples=2000, tol=0.005):
    """
    返回多解IK，用你的MDN模型预测。
    你只需保证 model() 和 forward_kinematics_from_angles() 可用即可。
    """
    import torch
    print(point)
    point = np.array(point, dtype=np.float32)
    with torch.no_grad():
        x = torch.tensor([point], dtype=torch.float32, device="cuda")
        pi, sigma, mu = model(x)
        samples = sample_from_mdn_multiple(pi, sigma, mu, num_samples=num_samples)[0]

    valid = []
    for s in samples:
        x,y = forward_kinematics_from_angles(*s)[-1]
        q1, q2, q3 = s
        if (np.linalg.norm(np.array([x,y]) - point) < tol) and (-np.pi/2 <= q3 <= np.pi/2):
            return [float(round(i,4)) for i in s]
            #valid.append([float(round(i,4)) for i in s])
            #break
    print(valid)
    return valid

class FB_RRTAngle:
    def __init__(self, tolerance=0.1, step_size=0.2):
        self.length = [0.325, 0.275, 0.265]  #[0.325, 0.275, 0.265]  #
        self.workspace_size = sum(self.length)
        #self.start_node = np.array(start_pos)
        self.tolerance = tolerance
        self.step_size = step_size
        #self.tree = {tuple(self.start_node): None}
        self.goal_node = None
        #self.obstacles = obstacles
        self.forward_kinematics_cache = {}
        #self.kd_tree = KDTree([self.start_node])

    def forward_kinematics(self,q):
        L1, L2, L3 = 0.325, 0.275, 0.265 #0.225
        t1, t2, t3 = q

        x0, y0 = 0, 0
        x1 = x0 + L1*np.cos(t1)
        y1 = y0 + L1*np.sin(t1)

        x2 = x1 + L2*np.cos(t1 + t2)
        y2 = y1 + L2*np.sin(t1 + t2)

        x3 = x2 + L3*np.cos(t1 + t2 + t3)
        y3 = y2 + L3*np.sin(t1 + t2 + t3)

        return np.array([[x0, x1, x2, x3],
                        [y0, y1, y2, y3]])


    # =======================================================
    #  Dummy IK (用于测试) —— 换成你的 MDN IK 即可
    # =======================================================
    def get_ik_dummy(self,x, y):
        """
        简单 IK，确保可运行。
        你替换成你的 MDN 版本即可。
        """

        # 计算末端距离
        dist = np.sqrt(x*x + y*y)

        # 3DOF 只有冗余，给一个简易的 IK 解
        t1 = np.arctan2(y, x)

        # 使手臂自然弯曲
        t2 = np.pi/4 * np.sin(dist * 3)
        t3 = -t2 / 2

        return [t1, t2, t3]


    # =======================================================
    #  碰撞检测
    # =======================================================
    def _line_circle_collision(self,x1, y1, x2, y2, cx, cy, r):
        d = np.array([x2 - x1, y2 - y1])
        f = np.array([x1 - cx, y1 - cy])
        a = np.dot(d, d)
        b = 2 * np.dot(f, d)
        c = np.dot(f, f) - r*r
        D = b*b - 4*a*c
        if D < 0:
            return False
        D = np.sqrt(D)
        t1 = (-b - D) / (2 * a)
        t2 = (-b + D) / (2 * a)
        return (0 <= t1 <= 1) or (0 <= t2 <= 1)


    def is_in_obstacle(self,q, obstacles):
        xs, ys = self.forward_kinematics(q)
        for x1, y1, x2, y2 in zip(xs, ys, xs[1:], ys[1:]):
            for (cx, cy, r) in obstacles:
                if self._line_circle_collision(x1, y1, x2, y2, cx, cy, r):
                    return True
        return False


    def find_collision_point_old(self,q_start, q_goal, obstacles, steps=300):
        q_start = np.array(q_start, dtype=float)
        q_goal  = np.array(q_goal,  dtype=float)

        for i in range(steps + 1):
            a = i / steps
            q = q_start + a * (q_goal - q_start)
            if self.is_in_obstacle(q, obstacles):
                return True, q
        return False, None
    def find_collision_point(self, q_start, q_goal, obstacles, resolution=0.05):
        """
        【改进版】检查从 q_start 到 q_goal 的路径是否发生碰撞。
        使用基于分辨率的方法，而不是固定的步数，使其更可靠。

        Args:
            q_start (list/np.array): 起始关节配置.
            q_goal (list/np.array): 目标关节配置.
            obstacles (list): 障碍物列表.
            resolution (float): 检查的分辨率（弧度）。值越小，检查越精细，也越慢。

        Returns:
            (bool, np.array or None): (是否碰撞, 第一个碰撞点的配置)
        """
        q_start = np.array(q_start, dtype=float)
        q_goal = np.array(q_goal, dtype=float)

        # 1. 计算C-Space中的距离
        delta = q_goal - q_start
        dist = np.linalg.norm(delta)

        # 如果距离非常小，可以认为没有移动，是安全的
        if dist < 1e-6:
            return False, None

        # 2. 根据距离和分辨率计算需要的步数
        # 确保至少检查起点和终点
        steps = max(2, int(dist / resolution))
        print(f"step:{steps}")

        # 3. 沿路径进行离散检查
        for i in range(steps + 1):
            # i=0 是起点，i=steps 是终点
            alpha = i / steps
            q_interpolated = q_start + alpha * delta

            # 检查插值点是否存在碰撞
            if self.is_in_obstacle(q_interpolated, obstacles):
                # 只要发现一个点碰撞，就证明整条路径无效
                return True, q_interpolated

                # 如果所有插值点都安全，则认为路径是无碰撞的
        return False, None

    # =======================================================
    #  关节路径规划（不用RRT）
    # =======================================================
    def plan_path(self,q_start, q_goal, obstacles,
                max_waypoints=500, step_size=0.1):

        def interp(q1, q2):
            q1 = np.array(q1); q2 = np.array(q2)
            dist = np.linalg.norm(q2 - q1)
            steps = max(2, int(dist / step_size))
            return [tuple(q1 + a*(q2 - q1)) for a in np.linspace(0,1,steps)]

        collided, q_col = self.find_collision_point(q_start, q_goal, obstacles)

        if not collided:
            print("Direct path OK")
            return interp(q_start, q_goal)

        path = []
        cur = np.array(q_start)
        i=0
        for _ in range(max_waypoints):
            i=i+1
            print("Collision occurred. Adding waypoint...",i)

            offset = np.random.uniform(-0.6, 0.6, size=3)
            wp = q_col + offset

            if self.is_in_obstacle(wp, obstacles):
                continue

            # cur → wp
            c1, col1 = self.find_collision_point(cur, wp, obstacles)
            if c1:
                continue

            # wp → goal
            c2, col2 = self.find_collision_point(wp, q_goal, obstacles)
            if c2:
                q_col = col2
                continue

            # OK
            path += interp(cur, wp)
            path += interp(wp, q_goal)
            return path

        print("Failed to avoid obstacle.")
        return None


    # =======================================================
    #  动画
    # =======================================================
    def animate_path(self,path, obstacles):
        fig, ax = plt.subplots()
        ax.set_aspect("equal")
        ax.grid(True)

        for (cx, cy, r) in obstacles:
            ax.add_patch(patches.Circle((cx, cy), r, alpha=0.3))

        reach = 0.325 + 0.275 + 0.265 + 0.1
        ax.set_xlim(-reach, reach)
        ax.set_ylim(-reach, reach)

        line, = ax.plot([], [], '-o', lw=3)

        def update(i):
            xs, ys = self.forward_kinematics(path[i])
            line.set_data(xs, ys)
            return line,

        ans=animation.FuncAnimation(fig, update, frames=len(path), interval=80, blit=True)
        plt.show()


    # =======================================================
    #  main(): 现在输入的是 XY，不是 joint
    # =======================================================
    def main(self,):

        # ------------ 你输入的是 WORLD 坐标 --------------
        start_xy = (0.75, 0.05)
        goal_xy  = (0.25, 0.35)

        # ------------ 转成 joint angle（用 IK） --------------
        q_start = get_ik(start_xy)
        q_goal  = get_ik(goal_xy)

        print("Start q:", q_start)
        print("Goal  q:", q_goal)

        # ------------ 障碍物（圆形） --------------
        obstacles = [
            (0.50, 0.18, 0.05),   # (cx, cy, r)
            (0.15, 0.50, 0.07)
        ]

        print("Planning...")
        path = self.plan_path(q_start, q_goal, obstacles)

        if path is None:
            print("No valid path!")
            return

        self.animate_path(path, obstacles)

model = IK_MDN_Model(input_size=2, output_size=3, num_gaussians=50)
model.load_state_dict(torch.load("/home/jimmy/Downloads/MDN_robot/ik_mdn_state.pt"))
model.to("cuda")
model.eval()
if __name__ == "__main__":
    rrt=FB_RRTAngle(start_pos=[0.0,0.0,0.0],obstacles=[(0.5, 0.18, 0.05),(0.15, 0.50, 0.07)],tolerance=0.05,step_size=0.2)
    rrt.main()
