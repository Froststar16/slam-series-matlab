import numpy as np
import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import (PoseWithCovarianceStamped, PoseStamped, Quaternion)
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


# ── Quaternion helpers (no tf_transformations needed) ─────────────────────

def quat_from_yaw(yaw: float):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (0.0, 0.0, sy, cy)   # x, y, z, w

def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


# ══════════════════════════════════════════════════════════════════════════
# EKF SLAM CORE
# ══════════════════════════════════════════════════════════════════════════

def ekf_predict(mu, sigma, v, w, dt, Q_robot):
    th = mu[2]
    n  = len(mu)
    mu = mu.copy()
    mu[0] += v * math.cos(th + w * dt / 2) * dt
    mu[1] += v * math.sin(th + w * dt / 2) * dt
    mu[2]  = wrap(mu[2] + w * dt)
    G = np.eye(n)
    G[0, 2] = -v * math.sin(th + w * dt / 2) * dt
    G[1, 2] =  v * math.cos(th + w * dt / 2) * dt
    Q_full = np.zeros((n, n))
    Q_full[:3, :3] = Q_robot
    sigma = G @ sigma @ G.T + Q_full
    return mu, sigma


def obs_model(rx, ry, rth, mx, my):
    dx, dy = mx - rx, my - ry
    return np.array([math.sqrt(dx**2 + dy**2),
                     wrap(math.atan2(dy, dx) - rth)])


def obs_jacobian(rx, ry, rth, mx, my):
    dx, dy = mx - rx, my - ry
    r2 = dx**2 + dy**2
    r  = math.sqrt(r2)
    return np.array([
        [-dx/r,  -dy/r,   0,  dx/r,  dy/r],
        [ dy/r2, -dx/r2, -1, -dy/r2, dx/r2]
    ])


def ekf_update(mu, sigma, observations, lm_registry, R, assoc_thresh=0.8):
    n_lm = (len(mu) - 3) // 2

    for (r_obs, phi_obs, lm_key) in observations:
        if lm_key in lm_registry:
            idx = lm_registry[lm_key]
        else:
            best_id, best_dist = -1, assoc_thresh
            for j in range(n_lm):
                mi = 3 + j * 2
                z_hat = obs_model(mu[0], mu[1], mu[2], mu[mi], mu[mi+1])
                innov = np.array([r_obs - z_hat[0], wrap(phi_obs - z_hat[1])])
                d = math.sqrt(innov[0]**2 + innov[1]**2 * 4)
                if d < best_dist:
                    best_dist, best_id = d, j

            if best_id >= 0:
                idx = best_id
                lm_registry[lm_key] = idx
            else:
                lm_x = mu[0] + r_obs * math.cos(phi_obs + mu[2])
                lm_y = mu[1] + r_obs * math.sin(phi_obs + mu[2])
                mu    = np.append(mu, [lm_x, lm_y])
                n     = len(mu)
                sigma_new = np.zeros((n, n))
                sigma_new[:n-2, :n-2] = sigma
                sigma_new[n-2, n-2]   = 1.0
                sigma_new[n-1, n-1]   = 1.0
                sigma = sigma_new
                idx   = n_lm
                n_lm += 1
                lm_registry[lm_key] = idx

        mi    = 3 + idx * 2
        z_hat = obs_model(mu[0], mu[1], mu[2], mu[mi], mu[mi+1])
        innov = np.array([r_obs - z_hat[0], wrap(phi_obs - z_hat[1])])
        H_small = obs_jacobian(mu[0], mu[1], mu[2], mu[mi], mu[mi+1])
        n = len(mu)
        H = np.zeros((2, n))
        H[:, :3]       = H_small[:, :3]
        H[:, mi:mi+2]  = H_small[:, 3:]
        S = H @ sigma @ H.T + R
        K = sigma @ H.T @ np.linalg.inv(S)
        mu    = mu + K @ innov
        mu[2] = wrap(mu[2])
        sigma = (np.eye(n) - K @ H) @ sigma

    return mu, sigma, lm_registry


def detect_landmarks(ranges, angles, min_r=0.05, max_r=3.5):
    r = np.array(ranges, dtype=float)
    valid = np.isfinite(r) & (r > min_r) & (r < max_r) & (r < max_r * 0.98)
    landmarks = []
    i = 1
    while i < len(r) - 1:
        if not valid[i]:
            i += 1
            continue
        # Find local minimum in a window
        if r[i] < r[i-1] and r[i] < r[i+1]:
            # Average over small window for stability
            w_start = max(0, i-2)
            w_end   = min(len(r)-1, i+2)
            r_avg   = np.mean(r[w_start:w_end+1])
            a_avg   = angles[i]
            # Stable key using coarse bins
            r_bin   = int(round(r_avg / 0.10))
            a_bin   = int(round(math.degrees(a_avg) / 5.0))
            key     = f"{r_bin}_{a_bin}"
            landmarks.append((float(r_avg), float(a_avg), key))
            i += 5  # skip ahead to avoid double-detecting same peak
        else:
            i += 1
    return landmarks


# ══════════════════════════════════════════════════════════════════════════
# ROS 2 NODE
# ═════════════════════════════════════════════════════════════════════════

class EKFSLAMNode(Node):

    def __init__(self):
        super().__init__('ekf_slam_node')

        self.declare_parameter('sig_v',           0.08)
        self.declare_parameter('sig_w',           0.04)
        self.declare_parameter('sig_r',           0.06)
        self.declare_parameter('sig_phi',         0.03)
        self.declare_parameter('assoc_thresh',    0.4)
        self.declare_parameter('lidar_max_range', 3.5)

        sig_v   = self.get_parameter('sig_v').value
        sig_w   = self.get_parameter('sig_w').value
        sig_r   = self.get_parameter('sig_r').value
        sig_phi = self.get_parameter('sig_phi').value

        self.Q = np.diag([sig_v**2, sig_v**2, sig_w**2])
        self.R = np.diag([sig_r**2, sig_phi**2])
        self.assoc_thresh    = self.get_parameter('assoc_thresh').value
        self.lidar_max_range = self.get_parameter('lidar_max_range').value

        self.mu    = np.array([0.0, 0.0, 0.0])
        self.sigma = np.zeros((3, 3))
        self.lm_registry: dict = {}
        self._last_odom_t  = None
        self._path_poses   = []

        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_subscription(Odometry,  '/odom', self._odom_cb, 10)

        self._pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/slam/pose', 10)
        self._lm_pub   = self.create_publisher(MarkerArray, '/slam/landmarks', 10)
        self._path_pub = self.create_publisher(Path, '/slam/path', 10)

        self.get_logger().info('EKF SLAM node ready — waiting for /scan and /odom')

    def _odom_cb(self, msg: Odometry):
        now = self.get_clock().now().nanoseconds * 1e-9
        v   = msg.twist.twist.linear.x
        w   = msg.twist.twist.angular.z
        if self._last_odom_t is not None:
            dt = now - self._last_odom_t
            if 0.001 < dt < 1.0:
                self.mu, self.sigma = ekf_predict(self.mu, self.sigma, v, w, dt, self.Q)
        self._last_odom_t = now

    def _scan_cb(self, msg: LaserScan):
        n      = len(msg.ranges)
        angles = [msg.angle_min + i * msg.angle_increment for i in range(n)]
        obs    = detect_landmarks(msg.ranges, angles,
                                  min_r=float(msg.range_min),
                                  max_r=self.lidar_max_range)
        if obs:
            self.mu, self.sigma, self.lm_registry = ekf_update(
                self.mu, self.sigma, obs, self.lm_registry, self.R, self.assoc_thresh)
        self._pub_pose()
        self._pub_landmarks()
        self._pub_path()

    def _pub_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(self.mu[0])
        msg.pose.pose.position.y = float(self.mu[1])
        qx, qy, qz, qw = quat_from_yaw(float(self.mu[2]))
        msg.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        cov = [0.0] * 36
        cov[0]  = float(self.sigma[0, 0])
        cov[1]  = float(self.sigma[0, 1])
        cov[6]  = float(self.sigma[1, 0])
        cov[7]  = float(self.sigma[1, 1])
        cov[35] = float(self.sigma[2, 2])
        msg.pose.covariance = cov
        self._pose_pub.publish(msg)

    def _pub_landmarks(self):
        ma  = MarkerArray()
        now = self.get_clock().now().to_msg()
        del_m = Marker()
        del_m.header.frame_id = 'map'
        del_m.header.stamp    = now
        del_m.action = Marker.DELETEALL
        ma.markers.append(del_m)
        n_lm = (len(self.mu) - 3) // 2
        for i in range(n_lm):
            mi = 3 + i * 2
            m  = Marker()
            m.header.frame_id = 'map'
            m.header.stamp    = now
            m.ns     = 'ekf_landmarks'
            m.id     = i
            m.type   = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(self.mu[mi])
            m.pose.position.y = float(self.mu[mi + 1])
            m.pose.position.z = 0.15
            m.pose.orientation.w = 1.0
            m.scale.x = 0.20
            m.scale.y = 0.20
            m.scale.z = 0.30
            m.color   = ColorRGBA(r=0.88, g=0.10, b=0.42, a=0.85)
            ma.markers.append(m)
        self._lm_pub.publish(ma)

    def _pub_path(self):
        ps = PoseStamped()
        ps.header.stamp    = self.get_clock().now().to_msg()
        ps.header.frame_id = 'map'
        ps.pose.position.x = float(self.mu[0])
        ps.pose.position.y = float(self.mu[1])
        qx, qy, qz, qw = quat_from_yaw(float(self.mu[2]))
        ps.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        self._path_poses.append(ps)
        path = Path()
        path.header.stamp    = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        path.poses = self._path_poses[-800:]
        self._path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = EKFSLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
