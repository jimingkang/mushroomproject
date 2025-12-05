import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib import animation

# ================================
#     你的 IK + MDN 部分
# ================================

def get_ik(point, num_samples=2000, tol=0.005):
    """
    返回多解IK，用你的MDN模型预测。
    你只需保证 model() 和 forward_kinematics_from_angles() 可用即可。
    """
    import torch
    point = np.array(point, dtype=np.float32)
    with torch.no_grad():
        x = torch.tensor([point], dtype=torch.float32, device="cuda")
        pi, sigma, mu = model(x)
        samples = sample_from_mdn_multiple(pi, sigma, mu, num_samples=num_samples)[0]

    valid = []
    for s in samples:
        x,y = forward_kinematics_from_angles(*s)[-1]
        if np.linalg.norm(np.array([x,y]) - point) < tol:
            valid.append([float(round(i,4)) for i in s])
    return valid


# ================================
#          RRT ANGLE 主类
# ================================

class RRTAngle:
    """
    多功能 RRT：
    - 支持 MDN IK bias
    - 支持碰撞点 waypoint bias
    - 支持动态动画
    """

    def __init__(self, start_pos, obstacles=None,
                 tolerance=0.01, step_size=0.2,
                 goal_sample_rate=0.3,
                 waypoint_sample_rate=0.3,
                 joint_limits=None):

        self.length = [0.325, 0.275, 0.225]
        self.start_node = np.array(start_pos, dtype=np.float32)

        self.tolerance = tolerance
        self.step_size = step_size
        self.obstacles = obstacles if obstacles else []

        # RRT tree
        self.tree = {tuple(self.start_node): None}
        self.nodes = [self.start_node.copy()]
        self.goal_node = None

        # bias pools
        self.ik_bias = []
        self.collision_waypoints = []

        # sampling parameters
        self.goal_sample_rate = goal_sample_rate
        self.waypoint_sample_rate = waypoint_sample_rate

        # FK cache
        self.fk_cache = {}

        # joint limits
        if joint_limits is None:
            self.joint_limits = np.array([
                [-np.pi/2,  np.pi/2],
                [-3*np.pi/4, 3*np.pi/4],
                [-3*np.pi/4, 3*np.pi/4]
            ])
        else:
            self.joint_limits = np.array(joint_limits)


    # -----------------------------
    # Forward kinematics
    # -----------------------------
    def forward_kinematics(self, q):
        key = tuple([round(float(i),4) for i in q])
        if key in self.fk_cache:
            return self.fk_cache[key]

        t1,t2,t3 = q
        x0,y0 = 0,0
        x1 = x0 + self.length[0]*np.cos(t1)
        y1 = y0 + self.length[0]*np.sin(t1)
        x2 = x1 + self.length[1]*np.cos(t1+t2)
        y2 = y1 + self.length[1]*np.sin(t1+t2)
        x3 = x2 + self.length[2]*np.cos(t1+t2+t3)
        y3 = y2 + self.length[2]*np.sin(t1+t2+t3)

        result = np.array([[x0,x1,x2,x3],
                           [y0,y1,y2,y3]], dtype=np.float32)
        self.fk_cache[key] = result
        return result


    # -----------------------------
    #   Obstacle Checking
    # -----------------------------
    def _line_circle_collision(self, x1,y1,x2,y2,cx,cy,r):
        d = np.array([x2-x1, y2-y1])
        f = np.array([x1-cx, y1-cy])
        a = np.dot(d,d)
        b = 2*np.dot(f,d)
        c = np.dot(f,f) - r*r
        D = b*b - 4*a*c
        if D<0: return False
        D = np.sqrt(D)
        t1 = (-b-D)/(2*a)
        t2 = (-b+D)/(2*a)
        return (0<=t1<=1) or (0<=t2<=1)

    def _line_rect_collision(self, x1,y1,x2,y2,rx,ry,rw,rh):
        for t in np.linspace(0,1,20):
            x = x1 + t*(x2-x1)
            y = y1 + t*(y2-y1)
            if rx<=x<=rx+rw and ry<=y<=ry+rh:
                return True
        return False

    def is_in_obstacle(self, q):
        xs, ys = self.forward_kinematics(q)
        for x1,y1,x2,y2 in zip(xs,ys,xs[1:],ys[1:]):
            for shape,params in self.obstacles:
                if shape=='circle':
                    cx,cy,r = params
                    if self._line_circle_collision(x1,y1,x2,y2,cx,cy,r):
                        return True
                elif shape=='rectangle':
                    x,y,w,h = params
                    if self._line_rect_collision(x1,y1,x2,y2,x,y,w,h):
                        return True
        return False


    # -----------------------------
    #  Sampling
    # -----------------------------
    def random_config(self):
        lo = self.joint_limits[:,0]
        hi = self.joint_limits[:,1]
        return lo + (hi-lo)*np.random.rand(3)

    def sample(self):
        r = np.random.rand()
        if self.ik_bias and r < self.goal_sample_rate:
            return np.array(self.ik_bias[np.random.randint(len(self.ik_bias))])

        if self.collision_waypoints and r < self.goal_sample_rate + self.waypoint_sample_rate:
            return np.array(self.collision_waypoints[np.random.randint(len(self.collision_waypoints))])

        return self.random_config()


    # -----------------------------
    #  Nearest + Steer
    # -----------------------------
    def nearest_node(self, q):
        d = [np.linalg.norm(n-q) for n in self.nodes]
        return self.nodes[int(np.argmin(d))]

    def steer(self, q_from, q_to):
        v = q_to - q_from
        dist = np.linalg.norm(v)
        if dist<self.step_size:
            return q_to
        return q_from + v/dist*self.step_size


    # -----------------------------
    #  Workspace distance
    # -----------------------------
    def distance_ws(self, q, goal_ws):
        xs, ys = self.forward_kinematics(q)
        ee = np.array([xs[-1], ys[-1]])
        return np.linalg.norm(ee - goal_ws)


    # -----------------------------
    #  Edge collision: for IK direct connect
    # -----------------------------
    def first_collision(self, q1, q2, steps=50):
        for i in range(steps):
            a = i/steps
            q = q1 + a*(q2-q1)
            if self.is_in_obstacle(q):
                return True, q
        return False, None


    # -----------------------------
    #  Main RRT
    # -----------------------------
    def build_tree(self, goal_ws, max_iter=800):
        goal_ws = np.array(goal_ws, dtype=np.float32)

        # ---- 1. 从 MDN 获取多解 IK ----
        self.ik_bias = get_ik(goal_ws)
        print("IK solutions:", len(self.ik_bias))

        # ---- 2. 尝试直接起点→IK解（无碰撞）----
        for ik in self.ik_bias:
            ik = np.array(ik)
            if self.is_in_obstacle(ik): continue

            collided, qcol = self.first_collision(self.start_node, ik)
            if not collided:
                print("Direct IK connection OK")
                return self.interpolate(self.start_node, ik)

            # 作为 waypoint
            self.collision_waypoints.append(qcol + np.random.uniform(-0.1,0.1,3))

        # ---- 3. 进入RRT ----
        for it in range(max_iter):
            s = self.sample()
            qn = self.nearest_node(s)
            new = self.steer(qn, s)

            if self.is_in_obstacle(new): continue

            new_t = tuple([float(x) for x in new])
            if new_t in self.tree: continue

            self.tree[new_t] = tuple([float(x) for x in qn])
            self.nodes.append(new)

            # check goal
            d = self.distance_ws(new, goal_ws)
            if d < self.tolerance:
                self.goal_node = new_t
                print("Reached goal at iter", it)
                return self.reconstruct()

        return None


    def reconstruct(self):
        path = []
        node = self.goal_node
        while node:
            path.append(node)
            node = self.tree[node]
        return path[::-1]

    def interpolate(self, q1, q2):
        dist = np.linalg.norm(q2-q1)
        steps = int(dist/self.step_size)+2
        path=[]
        for a in np.linspace(0,1,steps):
            q = q1 + a*(q2-q1)
            path.append(tuple(q))
        return path


    # =================================================
    #           动态动画（2D机械臂）
    # =================================================
    def draw_obstacles(self, ax):
        for shape,params in self.obstacles:
            if shape=='circle':
                cx,cy,r=params
                ax.add_patch(patches.Circle((cx,cy),r,alpha=0.3))
            elif shape=='rectangle':
                x,y,w,h=params
                ax.add_patch(patches.Rectangle((x,y),w,h,alpha=0.3))

    def animate(self, path, goal_ws=None, interval=120, save=None):
        fig, ax = plt.subplots()
        ax.set_aspect('equal')

        self.draw_obstacles(ax)

        # 轨迹线
        ee_x=[]
        ee_y=[]
        for q in path:
            xs,ys=self.forward_kinematics(q)
            ee_x.append(xs[-1])
            ee_y.append(ys[-1])
        ax.plot(ee_x, ee_y, '--', alpha=0.5)

        if goal_ws is not None:
            ax.scatter(goal_ws[0], goal_ws[1], c='magenta', marker='*', s=80)

        reach = sum(self.length)+0.1
        ax.set_xlim(-reach, reach)
        ax.set_ylim(-reach, reach)
        ax.grid(True)

        arm_line, = ax.plot([], [], '-o', lw=3)

        def init():
            arm_line.set_data([], [])
            return arm_line,

        def update(i):
            q=path[i]
            xs,ys=self.forward_kinematics(q)
            arm_line.set_data(xs, ys)
            return arm_line,

        ani = animation.FuncAnimation(fig, update, init_func=init,
                                      frames=len(path), interval=interval, blit=True)

        if save:
            ani.save(save, writer='ffmpeg')
        plt.show()
