#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Quaternion
import math, random

class SLAMFinal(Node):
    def __init__(self):
        super().__init__('slam_final')
        self.odom_pub = self.create_publisher(Odometry,    '/odom',           10)
        self.scan_pub = self.create_publisher(LaserScan,   '/scan',           10)
        self.path_pub = self.create_publisher(Path,        '/slam/path',      10)
        self.lm_pub   = self.create_publisher(MarkerArray, '/slam/landmarks', 10)
        self.timer    = self.create_timer(0.1, self.publish)
        self.x = 0.5; self.y = 0.5; self.th = math.pi/4
        self.t = 0; self.path_poses = []
        self.waypoints = [
            (1.0,1.0),(7.0,1.0),(7.0,8.0),(1.0,8.0),
            (1.0,1.0),(4.0,4.0),(7.0,1.0),(1.0,8.0),(7.0,8.0)
        ]
        self.wp_idx = 0
        self.landmarks = [
            (2.0,2.0),(4.0,1.0),(5.0,3.0),(3.0,5.0),(1.0,4.0),
            (6.0,2.0),(7.0,4.0),(5.0,6.0),(2.0,7.0),(7.0,7.0),
            (0.5,6.0),(4.0,8.0),(8.0,1.0),(8.0,5.0),(1.0,1.0)
        ]
        self.discovered = [False] * len(self.landmarks)
        self.get_logger().info('SLAM Final bridge started')

    def publish(self):
        self.t += 1
        if self.t == 1:
            clear = Path()
            clear.header.stamp    = self.get_clock().now().to_msg()
            clear.header.frame_id = 'map'
            clear.poses           = []
            self.path_pub.publish(clear)
            return

        wx, wy = self.waypoints[self.wp_idx % len(self.waypoints)]
        dx = wx - self.x; dy = wy - self.y
        dist = math.sqrt(dx**2 + dy**2)
        if dist < 0.25:
            self.wp_idx += 1
            nwx, nwy = self.waypoints[self.wp_idx % len(self.waypoints)]
            self.get_logger().info(f'Waypoint reached — next: ({nwx},{nwy})')

        target_th = math.atan2(dy, dx)
        dth = (target_th - self.th + math.pi) % (2*math.pi) - math.pi
        v   = min(0.4, dist) * 0.8 + random.gauss(0, 0.005)
        w   = dth * 2.0            + random.gauss(0, 0.004)
        self.x  += v * math.cos(self.th + w*0.05) * 0.1
        self.y  += v * math.sin(self.th + w*0.05) * 0.1
        self.th  = (self.th + w*0.1 + math.pi) % (2*math.pi) - math.pi

        odom = Odometry()
        odom.header.stamp         = self.get_clock().now().to_msg()
        odom.header.frame_id      = 'odom'
        odom.child_frame_id       = 'base_link'
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0
        odom.twist.twist.linear.x  = float(v)
        odom.twist.twist.angular.z = float(w)
        self.odom_pub.publish(odom)

        ranges = [4.0] * 360
        for (mx, my) in self.landmarks:
            ddx = mx - self.x; ddy = my - self.y
            r = math.sqrt(ddx**2 + ddy**2)
            if r > 3.5: continue
            angle = math.atan2(ddy, ddx) - self.th
            angle = (angle + math.pi) % (2*math.pi) - math.pi
            beam  = int(round(math.degrees(angle))) % 360
            ranges[beam] = min(ranges[beam], r + random.gauss(0, 0.01))

        scan = LaserScan()
        scan.header.stamp    = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_scan'
        scan.angle_min       = -math.pi
        scan.angle_max       =  math.pi
        scan.angle_increment = float(2*math.pi/360)
        scan.range_min       = 0.1
        scan.range_max       = 4.0
        scan.ranges          = [float(r) for r in ranges]
        self.scan_pub.publish(scan)

        ps = PoseStamped()
        ps.header.stamp    = self.get_clock().now().to_msg()
        ps.header.frame_id = 'map'
        ps.pose.position.x = float(self.x)
        ps.pose.position.y = float(self.y)
        ps.pose.position.z = 0.0
        cy = math.cos(self.th*0.5); sy = math.sin(self.th*0.5)
        ps.pose.orientation = Quaternion(x=0.0, y=0.0, z=float(sy), w=float(cy))
        self.path_poses.append(ps)

        path = Path()
        path.header.stamp    = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        path.poses           = self.path_poses[-1000:]
        self.path_pub.publish(path)

        ma  = MarkerArray()
        now = self.get_clock().now().to_msg()
        del_m = Marker()
        del_m.header.frame_id = 'map'
        del_m.header.stamp    = now
        del_m.action          = Marker.DELETEALL
        ma.markers.append(del_m)

        for i, (mx, my) in enumerate(self.landmarks):
            ddx = mx - self.x; ddy = my - self.y
            if math.sqrt(ddx**2 + ddy**2) < 3.5:
                self.discovered[i] = True
            if not self.discovered[i]: continue
            m = Marker()
            m.header.frame_id    = 'map'
            m.header.stamp       = now
            m.ns                 = 'landmarks'
            m.id                 = i
            m.type               = Marker.CYLINDER
            m.action             = Marker.ADD
            m.pose.position.x    = float(mx)
            m.pose.position.y    = float(my)
            m.pose.position.z    = 0.5
            m.pose.orientation.w = 1.0
            m.scale.x            = 0.3
            m.scale.y            = 0.3
            m.scale.z            = 1.0
            m.color.r            = 0.88
            m.color.g            = 0.10
            m.color.b            = 0.42
            m.color.a            = 0.9
            ma.markers.append(m)

        self.lm_pub.publish(ma)

        if self.t % 20 == 0:
            n = sum(self.discovered)
            self.get_logger().info(
                f'Step {self.t:3d} | pos=({self.x:.2f},{self.y:.2f}) | landmarks: {n}/15')

def main():
    rclpy.init()
    node = SLAMFinal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()