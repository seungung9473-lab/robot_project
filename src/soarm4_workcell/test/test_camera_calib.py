from pathlib import Path
import unittest
import numpy as np
import cv2
from soarm4_workcell.core import load, matrix
from soarm4_workcell.camera_calib import table_markers, solve, compare, detect, gate

P = Path(__file__).resolve().parents[1] / 'config/workcell.yaml'


def project(T_table_optical, K, pts):
    Tc = np.linalg.inv(T_table_optical); c = (Tc[:3, :3] @ pts.T + Tc[:3, 3:4]).T
    return (K @ c.T).T[:, :2] / c[:, 2:3], c[:, 2]


class CameraCalibTests(unittest.TestCase):
    def setUp(self):
        self.wc = load(P); self.markers = table_markers(self.wc)

    def synth(self, cam, noise=0.0, seed=0):
        T = matrix(self.wc['cameras'][cam]['T_table_optical']); K = np.array(self.wc['cameras'][cam]['camera_info']['K'], float)
        rng = np.random.default_rng(seed); frames = []
        for _ in range(3):
            f = {}
            for i, c in self.markers.items():
                uv, z = project(T, K, c); f[i] = uv + rng.normal(0, noise, uv.shape)
            frames.append(f)
        return T, K, frames

    def test_recovers_stored_extrinsics_with_pixel_noise(self):
        for cam in ('cam1', 'cam2'):
            T, K, frames = self.synth(cam, noise=0.3)
            Tn, q = solve(frames, self.markers, K)   # 초기값 없이
            c = compare(Tn, T)
            self.assertLess(c['shift_norm_mm'], 5.0, (cam, c)); self.assertLess(c['rotation_deg'], 0.3, (cam, c))
            self.assertLess(q['reproj_rms_px'], 1.0)

    def test_initial_guess_does_not_bias_result(self):
        T, K, frames = self.synth('cam2', noise=0.2)
        bad = T.copy(); bad[:3, 3] += [0.02, -0.02, 0.01]
        Tn, _ = solve(frames, self.markers, K, init_T_table_optical=bad)
        self.assertLess(compare(Tn, T)['shift_norm_mm'], 5.0)

    def test_detects_moved_camera(self):
        T, K, frames = self.synth('cam1', noise=0.0)
        moved = T.copy(); a = np.radians(1.0); R = np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]]); moved[:3, :3] = T[:3, :3] @ R
        frames_moved = []
        for _ in range(1):
            f = {}
            for i, c in self.markers.items(): f[i] = project(moved, K, c)[0]
            frames_moved.append(f)
        Tn, _ = solve(frames_moved, self.markers, K, init_T_table_optical=T)
        c = compare(Tn, T)
        self.assertGreater(c['rotation_deg'], 0.7); self.assertLess(c['rotation_deg'], 1.3)

    def test_marker_corner_convention(self):
        c = self.markers[10]; ctr = np.array(self.wc['fixed_markers']['centers_table_m']['10'], float)
        np.testing.assert_allclose(c.mean(0)[:2], ctr[:2], atol=1e-9)
        self.assertGreater(c[0, 1], c[3, 1])   # TL이 BL보다 +Y
        self.assertGreater(c[1, 0], c[0, 0])   # TR이 TL보다 +X
        self.assertAlmostEqual(np.linalg.norm(c[0] - c[1]), 0.07, places=6)

    def test_detect_rendered_markers(self):
        # 가상 사진에 마커를 그려 넣고 모서리 순서(TL,TR,BR,BL)가 맞게 검출되는지 확인
        T, K, _ = self.synth('cam2'); img = np.full((480, 640, 3), 255, np.uint8); d = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        drawn = {}
        for i, c in self.markers.items():
            uv, z = project(T, K, c)
            if not (np.all(z > 0) and np.all((uv[:, 0] > 5) & (uv[:, 0] < 635) & (uv[:, 1] > 5) & (uv[:, 1] < 475))): continue
            tex = cv2.aruco.drawMarker(d, i, 200); tex = cv2.copyMakeBorder(tex, 0, 0, 0, 0, cv2.BORDER_CONSTANT)
            src = np.float32([[0, 0], [199, 0], [199, 199], [0, 199]]); M = cv2.getPerspectiveTransform(src, uv.astype(np.float32))
            warped = cv2.warpPerspective(cv2.cvtColor(tex, cv2.COLOR_GRAY2BGR), M, (640, 480)); mask = cv2.warpPerspective(np.full((200, 200), 255, np.uint8), M, (640, 480)) > 0
            img[mask] = warped[mask]; drawn[i] = uv
        if not drawn: self.skipTest('가상 사진에 마커가 안 들어옴')
        found = detect(img, set(self.markers))
        self.assertTrue(set(found) & set(drawn))
        for i in set(found) & set(drawn): np.testing.assert_allclose(found[i], drawn[i], atol=2.5)

    def test_gate_rejects_false_detection_but_keeps_real_ones(self):
        T, K, frames = self.synth('cam2'); f = dict(frames[0]); f[99] = np.array([[10, 10], [60, 10], [60, 60], [10, 60]], float)  # 모니터 화면 같은 가짜
        m = dict(self.markers); m[99] = self.markers[10]
        ok, bad = gate(f, m, T, K)
        self.assertIn(99, bad); self.assertEqual(set(ok), set(self.markers))


if __name__ == '__main__':
    unittest.main()
