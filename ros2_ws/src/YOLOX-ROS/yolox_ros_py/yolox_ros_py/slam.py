import cv2
#import pangolin
import numpy as np
import OpenGL.GL as gl

from multiprocessing import Process, Queue
from skimage.measure import ransac
from skimage.transform import EssentialMatrixTransform

# ????,?????????????
class Mapp:
    def __init__(self, W, H):
        self.width  = W
        self.Height = H
        self.poses  = []
        self.points = []
        self.state = None
        self.q = Queue()

        #p = Process(target=self.viewer_thread, args=(self.q,))
        #p.daemon = True
        #p.start()

    def add_observation(self, pose, points):
        self.poses.append(pose)
        for point in points:
            self.points.append(point)

    def viewer_init(self):
        #pangolin.CreateWindowAndBind('Main', self.width, self.Height)
        gl.glEnable(gl.GL_DEPTH_TEST)

        #self.scam = pangolin.OpenGlRenderState(
            #pangolin.ProjectionMatrix(self.width, self.Height, 420, 420, self.width//2, self.Height//2, 0.2, 1000),
            #pangolin.ModelViewLookAt(0, -10, -8,
            #                         0,   0,  0,
            #                         0,  -1,  0))
        #self.handler = pangolin.Handler3D(self.scam)
        # Create Interactive View in window
        #self.dcam = pangolin.CreateDisplay()
        #self.dcam.SetBounds(0.0, 1.0, 0.0, 1.0, -self.width/self.Height)
        #self.dcam.SetHandler(self.handler)

    def viewer_thread(self, q):
        self.viewer_init()
        #while True:
        #    self.viewer_refresh(q)

    def viewer_refresh(self, q):
        if self.state is None or not q.empty():
            self.state = q.get()

        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        gl.glClearColor(1.0, 1.0, 1.0, 1.0)
        self.dcam.Activate(self.scam)

        # draw poses
        gl.glColor3f(0.0, 1.0, 0.0)
        #pangolin.DrawCameras(self.state[0])

        # draw keypoints
        gl.glPointSize(2)
        gl.glColor3f(1.0, 0.0, 0.0)
        #pangolin.DrawPoints(self.state[1])

        #pangolin.FinishFrame()

    def display(self):
        poses = np.array(self.poses)
        points = np.array(self.points)
        self.q.put((poses, points))



class Frame(object):
    idx = 0
    last_kps, last_des, last_pose = None, None, None

    def __init__(self, image):
        """
        ???????,Frame ???????????????
        """
        Frame.idx += 1

        self.image = image
        self.idx   = Frame.idx
        self.last_kps  = Frame.last_kps
        self.last_des  = Frame.last_des
        self.last_pose = Frame.last_pose


# ???????????????????
def normalize(K, pts):
    Kinv = np.linalg.inv(K)
    # turn [[x,y]] -> [[x,y,1]]
    add_ones = lambda x: np.concatenate([x, np.ones((x.shape[0], 1))], axis=1)
    norm_pts = np.dot(Kinv, add_ones(pts).T).T[:, 0:2]
    return norm_pts

# ?? orb ??
def extract_points(frame):
    orb = cv2.ORB_create()
    image = cv2.cvtColor(frame.image, cv2.COLOR_BGR2GRAY)
    # detection corners
    pts = cv2.goodFeaturesToTrack(image, 3000, qualityLevel=0.01, minDistance=3)
    # extract features
    kps = [cv2.KeyPoint(x=pt[0][0], y=pt[0][1], _size=20) for pt in pts]
    kps, des = orb.compute(image, kps)

    kps = np.array([(kp.pt[0], kp.pt[1]) for kp in kps])
    return kps, des

# ???????????????
def match_points(frame):
    bfmatch = cv2.BFMatcher(cv2.NORM_HAMMING)
    matches = bfmatch.knnMatch(frame.curr_des, frame.last_des, k=2)
    match_kps, idx1, idx2 = [], [], []

    for m,n in matches:
        if m.distance < 0.75*n.distance:
            idx1.append(m.queryIdx)
            idx2.append(m.trainIdx)

            p1 = frame.curr_kps[m.queryIdx]
            p2 = frame.last_kps[m.trainIdx]
            match_kps.append((p1, p2))
    assert len(match_kps) >= 8

    frame.curr_kps = frame.curr_kps[idx1]
    frame.last_kps = frame.last_kps[idx2]

    return match_kps

# ??????????
def fit_essential_matrix(match_kps):
    global K
    match_kps = np.array(match_kps)

    # ??????????????
    norm_curr_kps = normalize(K, match_kps[:, 0])
    norm_last_kps = normalize(K, match_kps[:, 1])

    # ???????????
    model, inliers = ransac((norm_last_kps, norm_curr_kps),
                            EssentialMatrixTransform,
                            min_samples=8,              # ???? 8 ??
                            residual_threshold=0.005,
                            max_trials=200)

    frame.curr_kps = frame.curr_kps[inliers]
    frame.last_kps = frame.last_kps[inliers]

    return model.params

# ????????????? R?t
def extract_Rt(E):
    W = np.mat([[0,-1,0],[1,0,0],[0,0,1]],dtype=float)
    U,d,Vt = np.linalg.svd(E)

    if np.linalg.det(U)  < 0: U  *= -1.0
    if np.linalg.det(Vt) < 0: Vt *= -1.0

    # ??????,?? R ????????? diag([1,1,1])
    R = (np.dot(np.dot(U, W), Vt))
    if np.sum(R.diagonal()) < 0:
        R = np.dot(np.dot(U, W.T), Vt)

    t = U[:, 2]     # ??????,?? t[2] > 0
    if t[2] < 0:
        t *= -1

    Rt = np.eye(4)
    Rt[:3, :3] = R
    Rt[:3, 3] = t
    return Rt          # Rt ?????????????????????

# opencv ???????
# def triangulate(pts1, pts2, pose1, pose2):
    # pts1 = normalize(pts1)
    # pts2 = normalize(pts2)

    # pose1 = np.linalg.inv(pose1)
    # pose2 = np.linalg.inv(pose2)

    # points4d = cv2.triangulatePoints(pose1[:3], pose2[:3], pts1.T, pts2.T).T
    # points4d /= points4d[:, 3:]
    # return points4d


# ???????????
def triangulate(pts1, pts2, pose1, pose2):
    global K
    pose1 = np.linalg.inv(pose1)            # ?????????????????, ????
    pose2 = np.linalg.inv(pose2)

    pts1 = normalize(K, pts1)                 # ??????????????
    pts2 = normalize(K, pts2)

    points4d = np.zeros((pts1.shape[0], 4))
    for i, (kp1, kp2) in enumerate(zip(pts1, pts2)):
        A = np.zeros((4,4))
        A[0] = kp1[0] * pose1[2] - pose1[0]
        A[1] = kp1[1] * pose1[2] - pose1[1]
        A[2] = kp2[0] * pose2[2] - pose2[0]
        A[3] = kp2[1] * pose2[2] - pose2[1]
        _, _, vt = np.linalg.svd(A)         # ? A ???????
        points4d[i] = vt[3]

    points4d /= points4d[:, 3:]            # ?????????? [x, y, z, 1]
    return points4d

# ?????????
def draw_points(frame):
    for kp1, kp2 in zip(frame.curr_kps, frame.last_kps):
        u1, v1 = int(kp1[0]), int(kp1[1])
        u2, v2 = int(kp2[0]), int(kp2[1])
        cv2.circle(frame.image, (u1, v1), color=(0,0,255), radius=3)
        cv2.line(frame.image, (u1, v1), (u2, v2), color=(255,0,0))
    return None

# ????
def check_points(points4d):
    # ??3D???????????
    good_points = points4d[:, 2] > 0
    # TODO: parallax??????????? ....
    return good_points

def process_frame(frame):
    # ??????????????
    frame.curr_kps, frame.curr_des = extract_points(frame)
    # ???????????????????????????????
    Frame.last_kps, Frame.last_des = frame.curr_kps, frame.curr_des

    if frame.idx == 1:
        # ?????????,?????????????
        frame.curr_pose = np.eye(4)
        points4d = [[0,0,0,1]]      # ??? [0, 0, 0] , 1 ????
    else:
        # ????, ???? RANSAC ???????
        match_kps = match_points(frame)
        print("frame: {}, curr_des: {}, last_des: {}, match_kps: {}".
            format(frame.idx, len(frame.curr_des), len(frame.last_des), len(match_kps)))
        # ????????????
        essential_matrix = fit_essential_matrix(match_kps)
        print("---------------- Essential Matrix ----------------")
        print(essential_matrix)
        # ?????????????? Rt
        Rt = extract_Rt(essential_matrix)
        # ?????????????????
        frame.curr_pose = np.dot(Rt, frame.last_pose)
        # ?????????????
        points4d = triangulate(frame.last_kps, frame.curr_kps, frame.last_pose, frame.curr_pose)

        good_pt4d = check_points(points4d)
        points4d = points4d[good_pt4d]
        # TODO: g2o ????
        #draw_points(frame)
    mapp.add_observation(frame.curr_pose, points4d)     # ???? pose ????????
    # ????? pose ????????? last_pose ??
    Frame.last_pose = frame.curr_pose
    return frame


if __name__ == "__main__":
    # camera intrinsics
    W, H = 960, 540
    F = 270
    K = np.array([[F,0,W//2],[0,F,H//2],[0,0,1]])
    mapp = Mapp(1024, 768)    # ????

    cap = cv2.VideoCapture("road.mp4")
    while cap.isOpened() :
        ret, image = cap.read()
        frame = Frame(image)

        if ret:
            frame = process_frame(frame)
        else:
            break

        cv2.imshow("slam", frame.image)
        mapp.display()
        if cv2.waitKey(30) & 0xFF == ord('q'): break