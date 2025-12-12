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


import numpy as np
from tqdm import tqdm
from scipy.spatial import KDTree
from matplotlib import pyplot as plt
import torch

class RRTAngle:
    def __init__(self, start_pos, obstacles=[], tolerance=0.1, step_size=0.2):
        self.length = [0.325, 0.275, 0.265] #0.225
        self.workspace_size = sum(self.length)
        self.start_node = np.array(start_pos)
        self.tolerance = tolerance
        self.step_size = step_size
        self.tree = {tuple(self.start_node): None}
        self.goal_node = None
        self.obstacles = obstacles
        self.forward_kinematics_cache = {}  
        self.kd_tree = KDTree([self.start_node])
    
    def add_obstacle(self, shape, params):
        self.obstacles.append((shape, params))

    def forward_kinematics(self, angles):
        angles_tuple = tuple([round(i,4) for i in angles])
        if angles_tuple in self.forward_kinematics_cache:
            return self.forward_kinematics_cache[angles_tuple]

        x0, y0 = 0, 0
        x1 = x0 + self.length[0] * np.cos(angles[0])
        y1 = y0 + self.length[0] * np.sin(angles[0])
        x2 = x1 + self.length[1] * np.cos(angles[0] + angles[1])
        y2 = y1 + self.length[1] * np.sin(angles[0] + angles[1])
        x3 = x2 + self.length[2] * np.cos(angles[0] + angles[1]+angles[2])
        y3 = y2 + self.length[2] * np.sin(angles[0] + angles[1]+angles[2])

        result = np.array([[x0, x1, x2,x3], [y0, y1, y2,y3]], dtype=np.float32)
        self.forward_kinematics_cache[angles_tuple] = result  
        return result

    def sample_position(self, ik_bais, goal_bias=0.2):
        if ik_bais is not None and np.random.rand() < goal_bias:
            index=np.random.choice([i for i in range(0,len(ik_bais))])
            return ik_bais[index]
        
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
        direction = np.array(to_node) - np.array(from_node)
        distance = np.linalg.norm(direction)
        if distance <= self.step_size:
            return np.array(to_node)
        direction /= distance
        return np.array(from_node) + self.step_size * direction

    def distance_in_workspace(self, angles, goal_ws):
        xs, ys = self.forward_kinematics(angles)
        end_effector = np.array([xs[-1], ys[-1]])
        distance = np.linalg.norm(end_effector - goal_ws)
        return distance

    def build_tree(self, goal_ws, max_iterations=1000,goal_bias=0.2):
        goal_ws = np.array(goal_ws, dtype=np.float32)
        ik_bais=get_ik(goal_ws)
        print("vaild ik sol:",len(ik_bais))
        for i in range(max_iterations):
            sample = self.sample_position(ik_bais,goal_bias if len(ik_bais)>0 else -1)
            nearest = self.nearest_node(sample)
            new_node = self.steer(nearest, sample)
            new_node_tuple = tuple(new_node)

            distance = self.distance_in_workspace(new_node, goal_ws)

            if new_node_tuple in self.tree:
                continue
            if self.is_in_obstacle(new_node):
                continue

            self.tree[new_node_tuple] = tuple(nearest)
            self.kd_tree = KDTree(list(self.tree.keys()))  # Rebuild the KDTree after adding new node

            if distance < self.tolerance:
                self.goal_node = new_node_tuple
                path= self.reconstruct_path()
                return path

        return None

    def reconstruct_path(self):
        path = []
        node = self.goal_node
        while node is not None:
            path.append(node)
            node = self.tree[node]
        return path[::-1]

model = IK_MDN_Model(input_size=2, output_size=3, num_gaussians=50)
model.load_state_dict(torch.load("/home/jimmy/Downloads/MDN_robot/ik_mdn_state.pt"))
model.to("cuda")
model.eval()
    