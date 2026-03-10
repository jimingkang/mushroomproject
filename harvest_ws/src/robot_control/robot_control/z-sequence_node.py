##updated 02/24/2026
#segmentation using Using uphill segmentation, indiviual tree finding algorthim 


from custom_interfaces.action import Motion
from custom_interfaces.msg import Mushroom,Mushrooms

from kneed import KneeLocator
import time
import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros

from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2


from sensor_msgs.msg import PointCloud2
import matplotlib.pyplot as plt

from scipy.spatial.transform import Rotation as R
from sklearn.cluster import DBSCAN

from kneed import KneeLocator
def find_threshold(points_np):
    y = [np.percentile(points_np[:,2],i) for i in range(1,101)]
    x = np.arange(len(y))
    knee_locator = KneeLocator(x, y, curve='convex', direction='increasing')
    knee = knee_locator.knee
    threshold=min(np.mean(points_np[:,2]),np.percentile(points_np[:,2],knee))
    return threshold*0.9

def db_filter(points,eps=0.015, min_samples=50):
    try:
        threshold=find_threshold(points)
        pts=points[points[:,2]>threshold]
        db = DBSCAN(eps=eps, min_samples=min_samples).fit(pts)
        labels = db.labels_
        return pts[labels>=0]
    except ValueError:
        print(len(pts))
        print("cannot filter no points")

import numpy as np
from scipy.spatial import KDTree
import time

def segment_points(points, k=100, max_xy_dist=0.0825, min_segment_size=50):
    """
    Segment a point cloud by flowing each point to its closest higher neighbor.
    """
    start_time=time.time()
    tree = KDTree(points[:, :2])  # XY only
    parent_set = set()
    child_to_parent_idx = {}

    # --- Pass 1: classify each point ---
    for i, pt in enumerate(points):
        dists, idxs = tree.query(pt[:2], k=k + 1)
        # Remove self index
        idxs = idxs[1:]
        dists = dists[1:]

        higher_mask = points[idxs, 2] > pt[2]
        if not np.any(higher_mask):
            if len(parent_set) == 0:  # ← Fix: was < 0
                parent_set.add(i)
                continue
            parent_distance = np.array([np.linalg.norm(points[item][:2] - points[i][:2]) for item in parent_set])
            if all(parent_distance > max_xy_dist/2.5):
                parent_set.add(i)
                print(points[i], "added as new parent")
                continue
            else:
                # Assign to the closest existing parent instead of orphaning the point
                closest_parent = list(parent_set)[np.argmin(parent_distance)]
                child_to_parent_idx[i] = closest_parent  # ← Fix: don't orphan it
                continue

        higher_idxs = idxs[higher_mask]
        higher_dists = dists[higher_mask]

        closest_idx = np.argmin(higher_dists)
        closest_higher_idx = higher_idxs[closest_idx]
        closest_dist = higher_dists[closest_idx]

        if closest_dist < max_xy_dist:
            child_to_parent_idx[i] = closest_higher_idx

    # --- Pass 2: trace each point to its root parent ---
    memo = {}

    def find_root(i):
        if i in memo:
            return memo[i]
        if i in parent_set or i not in child_to_parent_idx:
            memo[i] = i
            return i
        root = find_root(child_to_parent_idx[i])
        memo[i] = root
        return root

    point_root = np.array([find_root(i) for i in range(len(points))])

    # --- Pass 3: filter small segments ---
    unique_roots, counts = np.unique(point_root, return_counts=True)
    valid_roots = unique_roots[counts >= min_segment_size]

    valid_segments = sorted(
        [points[point_root == root] for root in valid_roots],
        key=len,
        reverse=True,
    )

    parent_points = points[list(parent_set)]

    # print(f"Total segments   : {len(unique_roots)}")
    # print(f"Valid segments   : {len(valid_segments)}")
    # print(f"Removed segments : {len(unique_roots) - len(valid_segments)}")
    # print(f"Run Time: {time.time()-start_time}")
    return valid_segments, parent_points
def calc_pickipbility(mushroom_points,peaks):
    mean_peaks=[np.mean(peaks[:,0]),np.mean(peaks[:,1])]
    areas=[]
    heights=[]
    distances=[]
    if len(mushroom_points)==1:
        return [1.0]
    for mushroom_point,peak in zip(mushroom_points,peaks):
        x=mushroom_point[:,0]
        y=mushroom_point[:,1]
        z=mushroom_point[:,2]
        r1=max(x)-min(x)
        r2=max(y)-min(y)
        area=r1*r2
        
        height=peak[-1]
        distance=np.linalg.norm(mean_peaks-peak[:2])
        areas.append(area)
        heights.append(height)
        distances.append(distance)
    areas=np.array(areas)
    areas=areas/max(areas)
    heights=np.array(heights)
    heights=heights/max(heights)
    distances=np.array(distances)
    distances=distances/max(distances)
    score=0.1*areas+0.2*distances+0.35*heights
    return score/np.sum(score)


# from kneed import KneeLocator
# from sklearn.cluster import DBSCAN
# def find_threshold(points_np):
#     y = [np.percentile(points_np[:,2],i) for i in range(1,101)]
#     x = np.arange(len(y))
#     knee_locator = KneeLocator(x, y, curve='convex', direction='increasing')
#     knee = knee_locator.knee
#     threshold=min(np.mean(points_np[:,2]),np.percentile(points_np[:,2],knee))
#     return threshold*0.9

# def db_filter(points):
#     threshold=find_threshold(points)
#     pts=points[points[:,2]>threshold]
#     db = DBSCAN(eps=0.05, min_samples=100).fit(pts)
#     labels = db.labels_
#     return pts[labels>=0]