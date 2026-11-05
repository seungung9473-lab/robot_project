"""복원한 점군·흡착 후보·최종 흡착 위치를 RViz(/workcell_markers, 프레임 table)에 띄운다. 표시만, 모터·펌프 명령 없음.

사용:
    python3 -m soarm4_workcell.pickup_markers --cloud cloud.npz --result pickup.json
Ctrl+C로 끝내면 RViz에서 사라진다 (네임스페이스 pickup_*).
"""
import argparse, json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

CUP_R = 0.0125


class PickupMarkers(Node):
    def __init__(self, cloud, result, max_points=9000):
        super().__init__('pickup_markers'); self.result = result
        data = np.load(cloud)
        P = data['points']; step = max(1, len(P) // max_points); self.P = P[::step]
        self.rgb = None
        if 'colors_rgb' in data:
            rgb = np.asarray(data['colors_rgb'], dtype=float)
            if rgb.shape != P.shape or not np.isfinite(rgb).all():
                raise ValueError('colors_rgb must match finite Nx3 points')
            self.rgb = np.clip(rgb[::step] / 255.0, 0.0, 1.0)
        else:
            from pathlib import Path
            color_file = Path(cloud).with_name('color_classification.json')
            if color_file.exists():
                stats = json.loads(color_file.read_text())
                samples = [v['median_rgb'] for c in stats['cameras'] for v in c['views']]
                self.rgb = np.tile(np.median(samples, axis=0) / 255.0, (len(self.P), 1))
        qos = QoSProfile(depth=5, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(MarkerArray, 'workcell_markers', qos); self.create_timer(1.0, self.publish); self.publish()

    def marker(self, ns, i, typ, color, scale=None):
        m = Marker(); m.header.frame_id = 'table'; m.ns = ns; m.id = i; m.type = typ; m.action = Marker.ADD; m.frame_locked = True
        m.pose.orientation.w = 1.0; m.color.r, m.color.g, m.color.b, m.color.a = color
        if scale: m.scale.x, m.scale.y, m.scale.z = scale
        return m

    def circle(self, ns, i, c, z, color, r=CUP_R, width=0.0012):
        m = self.marker(ns, i, Marker.LINE_STRIP, color, (width, 0.0, 0.0))
        for t in np.linspace(0, 2 * np.pi, 41):
            m.points.append(Point(x=float(c[0] + r * np.cos(t)), y=float(c[1] + r * np.sin(t)), z=float(z + 0.0008)))
        return m

    def publish(self):
        r = self.result; arr = MarkerArray(); zt = r['plane_z_m']
        cloud = self.marker('pickup_cloud', 0, Marker.POINTS, (0.85, 0.2, 0.2, 1.0), (0.0025, 0.0025, 0.0))
        zs = self.P[:, 2]; shade = (zs - zs.min()) / max(1e-6, np.ptp(zs))
        from std_msgs.msg import ColorRGBA
        for idx, (p, s) in enumerate(zip(self.P, shade)):
            cloud.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
            rgb = self.rgb[idx] if self.rgb is not None else (0.6, 0.6, 0.6)
            cloud.colors.append(ColorRGBA(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=1.0))
        arr.markers.append(cloud)
        for i, (lbl, c) in enumerate(r['candidates'].items()):
            ok = r['valid'].get(lbl, False); chosen = lbl == r['candidate']
            col = (0.1, 0.9, 0.2, 1.0) if chosen else ((1.0, 0.85, 0.0, 1.0) if ok else (1.0, 0.2, 0.2, 1.0))
            arr.markers.append(self.circle('pickup_candidates', i, c, zt, col, width=0.0025 if chosen else 0.0012))
            t = self.marker('pickup_labels', i, Marker.TEXT_VIEW_FACING, col, (0.0, 0.0, 0.012)); t.pose.position.x, t.pose.position.y, t.pose.position.z = c[0] + 0.014, c[1] + 0.014, zt + 0.004; t.text = lbl; arr.markers.append(t)
        p = r['pickup_table_m']
        cup = self.marker('pickup_best', 0, Marker.CYLINDER, (0.1, 0.9, 0.2, 0.55), (2 * CUP_R, 2 * CUP_R, 0.002)); cup.pose.position.x, cup.pose.position.y, cup.pose.position.z = p[0], p[1], p[2] + 0.001; arr.markers.append(cup)
        arrow = self.marker('pickup_best', 1, Marker.ARROW, (0.1, 0.9, 0.2, 1.0), (0.004, 0.009, 0.014)); arrow.points = [Point(x=p[0], y=p[1], z=p[2] + 0.06), Point(x=p[0], y=p[1], z=p[2] + 0.004)]; arr.markers.append(arrow)
        txt = self.marker('pickup_best', 2, Marker.TEXT_VIEW_FACING, (0.05, 0.5, 0.1, 1.0), (0.0, 0.0, 0.016)); txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = p[0], p[1] - 0.03, p[2] + 0.075
        txt.text = f"pickup ({p[0]*100:.1f}, {p[1]*100:.1f}) cm  z {p[2]*1000:.0f} mm"; arr.markers.append(txt)
        self.pub.publish(arr)


def main(argv=None):
    ap = argparse.ArgumentParser(description='점군·흡착 위치를 RViz에 표시 (표시만)'); ap.add_argument('--cloud', required=True); ap.add_argument('--result', required=True)
    a = ap.parse_args(argv); rclpy.init(); n = PickupMarkers(a.cloud, json.load(open(a.result)))
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':main()
