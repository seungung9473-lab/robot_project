"""Read-only audience view of the current SO-ARM demo and its saved evidence."""
import argparse
from datetime import datetime
import fcntl
import json
from pathlib import Path
import time
import tkinter as tk

from PIL import Image, ImageDraw, ImageTk

import demo_scene

RUNS = Path(__file__).resolve().parents[1] / 'test_data/runs'
BG, CARD, FG, MUTED, ACCENT = '#101826', '#1b293b', '#f3f7fc', '#a9bed2', '#59e4ca'
COLORS = ('#59e4ca', '#f4bf6b', '#83a8ff', '#f293b5')
STEPS = ('RGB-D 스캔', '물체 분할·복원', '날짜 판독·분류', '경로 검사·집기')


def read_json(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return {}


def recent_demo(since):
    for run in sorted(RUNS.glob('*'), reverse=True):
        summary = read_json(run / 'summary.json')
        if summary.get('kind') != 'natural_language_tidy_demo' or not summary.get('physical_execution_requested'):
            continue
        start = datetime.fromisoformat(summary['started_at']).timestamp()
        status = run / 'current_status.json'
        active = (summary.get('outcome') == 'running' and status.is_file()
                  and status.stat().st_mtime >= time.time() - 30)
        if start >= since or active:
            return run
        break
    return None


def stage_index(name):
    if any(word in name for word in ('경로 검토', '빈자리')):
        return 3
    if any(word in name for word in ('판독', '스티커', '날짜 분류')):
        return 2
    if any(word in name for word in ('SAM2', '물체 식별')):
        return 1
    if '스캔·0도' in name:
        return 0
    return -1


def scan_folder(run):
    sessions = list(run.glob('scans/*/session.json')) + list(run.glob('rescan_*/*/session.json'))
    return max(sessions, key=lambda p: p.parent.name).parent if sessions else None


def progress_detail(run, index, pad_names):
    if index != 3:
        return ''
    try:
        with (run / 'execution.log').open('rb') as stream:
            stream.seek(max(0, stream.seek(0, 2) - 16000))
            lines = stream.read().decode('utf-8', errors='replace').splitlines()
    except OSError:
        return ''
    for line in reversed(lines):
        if line.startswith('BATCH STAGE '):
            stage, _, pad = line.partition(' -> ID ')
            destination = '폐기' if pad == 'None' else pad_names.get(pad, '배치')
            return stage.replace('BATCH STAGE', '집기 단계').replace('object_', '물체 ') + ' → ' + destination
        if line.startswith('계산중: object_') and '후보' in line:
            return line[:105].replace('object_', '물체 ')
    return '검토한 경로가 확보되면 집기를 시작합니다'


class LiveCamera:
    def __init__(self):
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image as ImageMessage
        from std_msgs.msg import Int32

        self.ros = rclpy
        self.frame = None
        self.received = 0
        self.pressure = None
        self.pressure_received = 0
        rclpy.init(domain_id=73)
        self.node = rclpy.create_node('soarm_demo_monitor_camera')
        self.subscription = self.node.create_subscription(
            ImageMessage, '/camera/cam1/color/image_raw', self.receive,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.pressure_subscription = self.node.create_subscription(
            Int32, '/vacuum/diff', self.receive_pressure,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def receive_pressure(self, message):
        self.pressure = message.data
        self.pressure_received = time.monotonic()

    def poll(self):
        for _ in range(2):
            self.ros.spin_once(self.node, timeout_sec=0)
        if time.monotonic() - self.pressure_received < 1:
            return self.pressure
        return None

    def receive(self, message):
        mode = {'rgb8': 'RGB', 'bgr8': 'BGR'}.get(message.encoding)
        if mode:
            self.frame = Image.frombytes(
                'RGB', (message.width, message.height), bytes(message.data), 'raw', mode, message.step)
            self.received = time.monotonic()

    def latest(self):
        if time.monotonic() - self.received < 1:
            return self.frame

    def close(self):
        self.node.destroy_node()
        self.ros.try_shutdown()


class Monitor:
    def __init__(self, root, replay, camera=None):
        self.root, self.replay, self.since = root, replay, time.time() - 5
        self.run = replay
        self.camera = camera
        self.live_requested = False
        self.image_key = None
        self.preview_key = None
        self.preview_started = 0.0
        self.photos = []
        root.title('TUMST ARM · 실시간 인식과 분류')
        root.geometry('1240x800')
        root.minsize(880, 580)
        root.configure(bg=BG)
        root.bind('<F11>', lambda _: root.attributes('-fullscreen', not root.attributes('-fullscreen')))
        root.bind('<Escape>', lambda _: root.attributes('-fullscreen', False))
        top = tk.Frame(root, bg=BG)
        top.pack(fill='x', padx=26, pady=(19, 12))
        self.label(top, 'TUMST ARM  /  시연 모니터', 24, bold=True).pack(side='left')
        self.clock = self.label(top, '00:00', 24, color=ACCENT, bold=True)
        self.clock.pack(side='right')
        self.pressure_label = self.label(top, '압력 diff · 대기', 14, color=MUTED, bold=True)
        if not replay:
            self.pressure_label.pack(side='right', padx=(0, 22))

        bar = tk.Frame(root, bg=BG)
        bar.pack(fill='x', padx=26, pady=(0, 10))
        self.steps = []
        for i, name in enumerate(STEPS, 1):
            chip = self.label(bar, f'{i:02d}   {name}', 13, bg=CARD, color=MUTED, bold=True)
            chip.pack(side='left', fill='x', expand=True, padx=(0, 6), ipady=9)
            self.steps.append(chip)

        self.title = self.label(root, '시연 시작 대기', 20, bold=True)
        self.title.pack(anchor='w', padx=26)
        self.detail = self.label(root, '터미널에서 분류 시연을 시작하면 이 화면이 자동 갱신됩니다.', 12, color=MUTED)
        self.detail.pack(anchor='w', padx=26, pady=(3, 10))

        self.cameras = tk.Frame(root, bg=BG)
        self.cameras.pack(fill='both', expand=True, padx=26)
        self.views = []
        self.camera_titles = []
        for i, text in enumerate(('CAM 1  /  상단 시점', 'CAM 2  /  측면 시점')):
            self.cameras.columnconfigure(i, weight=1, uniform='camera')
            self.cameras.rowconfigure(0, weight=1)
            card = tk.Frame(self.cameras, bg=CARD)
            card.grid(row=0, column=i, sticky='nsew', padx=(0, 8) if i == 0 else (8, 0))
            title = self.label(card, text, 12, bg=CARD, color=ACCENT, bold=True)
            title.pack(anchor='w', padx=12, pady=7)
            self.camera_titles.append(title)
            view = tk.Canvas(card, bg=CARD, highlightthickness=0)
            view.pack(fill='both', expand=True)
            self.views.append(view)
        self.results = tk.Frame(root, bg=BG)
        self.results.pack(fill='x', padx=26, pady=(12, 7))
        self.result_lines = [self.label(self.results, '', 12, color=MUTED) for _ in range(4)]
        for line in self.result_lines:
            line.pack(anchor='w')
        self.label(root, 'F11 전체 화면  ·  Esc 해제', 10, color=MUTED).pack(anchor='e', padx=26, pady=(0, 9))
        root.after(100, self.refresh)

    def label(self, parent, text, size, bg=BG, color=FG, bold=False):
        return tk.Label(parent, text=text, bg=bg, fg=color,
                        font=('sans-serif', size, 'bold' if bold else 'normal'), anchor='w')

    def draw_images(self, scan, view, masks, boxes, scene=None, *, live=False):
        self.live_requested = live and self.camera is not None
        frame = self.camera.latest() if self.live_requested else None
        sizes = [(c.winfo_width(), c.winfo_height()) for c in self.views]
        mask_files = tuple((scan/'objects'/obj/'sam2'/view/f'cam{index}_mask.png', color)
                           for obj, color in masks for index in (1, 2))
        key = (scan, view, tuple((path.is_file(), color) for path, color in mask_files),
               masks, tuple(str(b) for b in boxes), sizes, self.camera.received if frame is not None else None)
        if scene is not None:
            key += (tuple(len(xyz) for xyz, _ in scene[0]), str(scene[1]), len(scene[2]), scene[3])
            if scene[2]:
                key += (int(time.monotonic()*4),)
        if key == self.image_key:
            return
        self.image_key = key
        self.photos = []
        for index, (canvas, (width, height)) in enumerate(zip(self.views, sizes), 1):
            canvas.delete('all')
            if index == 1:
                self.camera_titles[0].configure(text='CAM 1  /  실시간 컬러 영상' if frame is not None
                                                else 'CAM 1  /  상단 시점 · 촬영 기록')
                if frame is not None:
                    image = frame.copy()
                    image.thumbnail((max(width-10, 1), max(height-10, 1)), Image.Resampling.BILINEAR)
                    photo = ImageTk.PhotoImage(image)
                    self.photos.append(photo)
                    canvas.create_image(width//2, height//2, image=photo)
                    continue
            if index == 2:
                title = 'CAM 2  /  측면 시점'
                if scene is not None:
                    clouds, contacts, paths, active = scene
                    preview_key = (scan, active)
                    now = time.monotonic()
                    if preview_key != self.preview_key:
                        self.preview_key = preview_key
                        self.preview_started = now
                    robot = demo_scene.preview_pose(paths, now - self.preview_started)
                    image = demo_scene.render(max(width, 1), max(height, 1), clouds, contacts, paths, robot)
                    if image:
                        title = ('검증 경로 · ' + active.replace('object_', '물체 ') if paths
                                 else '3D 복원 · 흡착 후보')
                        self.camera_titles[1].configure(text=title)
                        photo = ImageTk.PhotoImage(image)
                        self.photos.append(photo)
                        canvas.create_image(width//2, height//2, image=photo)
                        legend = 'SO-ARM 계획 자세  ·  파랑  흡착컵 경로  ·  노랑  흡착점' if paths else '색상  물체별 점군    ·    노랑  흡착 후보'
                        canvas.create_text(12, height-13, text=legend, anchor='sw', fill=FG, font=('sans-serif', 11))
                        continue
                self.camera_titles[1].configure(text=title)
            image_path = scan / view / f'cam{index}_color.png' if scan and view else None
            try:
                if not image_path:
                    raise FileNotFoundError()
                with Image.open(image_path) as original:
                    image = original.convert('RGB')
                for obj, color in masks:
                    path = scan/'objects'/obj/'sam2'/view/f'cam{index}_mask.png'
                    if path.is_file():
                        with Image.open(path) as mask:
                            alpha = mask.convert('L').point(lambda x: 95 if x > 127 else 0)
                        if alpha.size == image.size:
                            image.paste(color, (0, 0, *image.size), alpha)
                if index == 1 and not masks:
                    draw = ImageDraw.Draw(image)
                    for j, obj in enumerate(boxes):
                        box = obj.get('box_full_image_xyxy')
                        if box and len(box) == 4:
                            draw.rectangle(box, outline=COLORS[j % len(COLORS)], width=3)
                image.thumbnail((max(width-10, 1), max(height-10, 1)), Image.Resampling.LANCZOS)
                photo = ImageTk.PhotoImage(image)
                self.photos.append(photo)
                canvas.create_image(width//2, height//2, image=photo)
            except OSError:
                canvas.create_text(width//2, height//2, text='촬영 이미지 대기', fill=MUTED, font=('sans-serif', 14))
                if image_path:
                    self.image_key = None

    def refresh(self):
        try:
            self.update_screen()
        except (OSError, ValueError, KeyError):
            # Writers may be replacing a report as the monitor reads it.
            self.image_key = None
        self.root.after(125 if self.live_requested else 250, self.refresh)

    def update_screen(self):
        if self.camera is not None:
            diff = self.camera.poll()
            self.pressure_label.configure(text='압력 diff · 수신 없음' if diff is None else f'압력 diff  {diff}',
                                          fg=MUTED if diff is None else ACCENT)
        self.run = self.replay or recent_demo(self.since) or self.run
        if not self.run:
            self.draw_images(None, None, (), [])
            return
        run = self.run
        status = read_json(run/'current_status.json')
        result = read_json(run/'demo_result.json')
        summary = read_json(run/'summary.json')
        scan = scan_folder(run)
        name = status.get('stage', '시연 준비')
        idx = stage_index(name)
        done = summary.get('outcome') == 'complete'
        failed = summary.get('outcome') in ('stopped', 'failed', 'interrupted')
        total = result.get('total_seconds', status.get('total_seconds', 0))
        if summary.get('outcome') == 'running':
            total = max(float(total or 0), time.time()-datetime.fromisoformat(summary['started_at']).timestamp())
        minutes, seconds = divmod(max(0, int(total or 0)), 60)
        self.clock.configure(text=f'{minutes:02d}:{seconds:02d}')
        prefix = '기록 재생  /  ' if self.replay else ''
        self.title.configure(text=prefix+('분류 완료' if done else '시연 중단' if failed else name))
        for i, chip in enumerate(self.steps):
            chip.configure(fg=BG if idx == i and not failed else ACCENT if i < idx or done else MUTED,
                           bg=ACCENT if idx == i and not failed else CARD)

        lines, masks, boxes = [], (), []
        view = None
        labels = read_json(scan/'pad_labels.json') if scan else {}
        if scan and not labels:
            recorded = read_json(scan/'expiry_route_report.json').get('pad_labels')
            if isinstance(recorded, dict):
                labels = recorded
            elif recorded == 'default_fixed_ID5_discount_ID6_regular':
                labels = {'discount_pad_id': 5, 'regular_pad_id': 6}
        pad_names = {str(labels[key]): name for key, name in
                     (('discount_pad_id', '할인'), ('regular_pad_id', '정규')) if labels.get(key) is not None}
        detail = progress_detail(run, idx, pad_names)
        if scan:
            manifest = read_json(scan/'session.json')
            views = manifest.get('views', [])
            detection = read_json(scan/'gemini_detection.json')
            boxes = detection.get('objects', [])
            if views:
                view = views[0]['path'] if boxes else views[-1]['path']
            elif manifest.get('baseline'):
                view = manifest['baseline']
            objects = sorted((scan/'objects').glob('object_*'))
            masks = tuple((o.name, COLORS[i % len(COLORS)]) for i, o in enumerate(objects)
                          if view and any((o/'sam2'/view).glob('*_mask.png')))
            dates = read_json(scan/'expiry_observations.json')
            routes = read_json(scan/'color_sort_plan.json').get('objects', [])
            if idx == 0:
                degrees = views[-1].get('angle_deg') if views else None
                detail = f'정지 촬영 {len(views)} / {len(manifest.get("angles", [])) or 12}'
                if degrees is not None:
                    detail += f'   ·   턴테이블 {degrees:.0f}°'
            elif idx == 1:
                reconstructed = sum((o/'analysis/cloud.npz').is_file() for o in objects)
                detail = f'Gemini 물체 {len(boxes)}개  ·  SAM2 마스크 {len(masks)}개  ·  3D 복원 {reconstructed}개'
            elif idx == 2:
                read_count = len(dates.get('objects', []))
                if not read_count:
                    responses = {p.name.split('_response.json')[0] for p in scan.glob('numeric_dates_*/**/object_*_response.json')
                                 if '_latest_' not in p.name}
                    read_count = len(responses)
                detail = f'날짜 응답 수신 물체 {read_count} / {len(objects) or len(boxes)}'
            if routes:
                for o in routes[:4]:
                    decision = o.get('expiry_decision', {})
                    date_text = decision.get('expiry_date') or '날짜 확인 중'
                    destination = o.get('destination')
                    pad = o.get('destination_pad_id')
                    route = '폐기' if destination == 'trash_drop' else pad_names.get(str(pad), '배치 위치 확인 중') if pad else '분류 대기'
                    lines.append(f'{o.get("object", "").replace("object_", "물체 ")}   {o.get("label", "")}   ·   {date_text}   ·   {route}')
            elif boxes:
                for i, obj in enumerate(boxes[:4], 1):
                    lines.append(f'물체 {i}   {obj.get("label", "")}')
            scene = demo_scene.geometry(scan, COLORS, idx == 3 or done or failed) if idx >= 1 or done or failed else None
            self.draw_images(scan, view, masks, boxes, scene, live=idx >= 2 or done or failed)
        else:
            self.draw_images(None, None, (), [])
        if done:
            detail = '배치 완료'
        elif failed:
            detail = (result.get('error') or status.get('error') or '실행이 중단됐습니다.').splitlines()[0][:120]
        self.detail.configure(text=detail or '처리 중')
        for i, line in enumerate(self.result_lines):
            line.configure(text=lines[i] if i < len(lines) else '')


def main():
    parser = argparse.ArgumentParser(description='시연 상태와 카메라·인식 결과를 보는 창')
    parser.add_argument('--run', type=Path, help='특정 실행 기록을 표시합니다')
    args = parser.parse_args()
    lock = None
    if args.run is None:
        lock = (RUNS.parent/'.demo_monitor.lock').open('a')
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return
    root = tk.Tk()
    camera = None
    try:
        if args.run is None:
            try:
                camera = LiveCamera()
            except ImportError:
                print('ROS 환경을 불러오면 CAM 1 실시간 영상을 볼 수 있습니다.', flush=True)
        Monitor(root, args.run, camera)
        root.mainloop()
    finally:
        if camera:
            camera.close()
        if lock:
            lock.close()


if __name__ == '__main__':
    main()
