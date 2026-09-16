import numpy as np

from edge_perception.bunds import detect_bunds
from edge_perception.ground import estimate_ground_grid
from edge_perception.rocks import detect_rocks
from edge_perception.vibration import ImuSample, VibrationMonitor


def test_ground_and_rocks():
    rng = np.random.default_rng(0)
    ground = np.column_stack(
        [
            rng.uniform(5, 25, 400),
            rng.uniform(-3, 3, 400),
            rng.normal(0.0, 0.05, 400),
        ]
    )
    rock = np.column_stack(
        [
            12.0 + rng.normal(0, 0.15, 40),
            1.0 + rng.normal(0, 0.15, 40),
            0.4 + rng.normal(0, 0.1, 40),
        ]
    )
    pts = np.vstack([ground, rock]).astype(np.float32)
    gr = estimate_ground_grid(pts)
    assert gr.ground_idx.size > 0
    obs = pts[gr.obstacle_idx]
    hag = gr.hag[gr.obstacle_idx]
    dets = detect_rocks(obs, hag)
    assert len(dets) == 1
    assert dets[0].type == 'rock'
    assert abs(dets[0].x - 12.0) < 1.0
    assert abs(dets[0].y - 1.0) < 1.0


def test_bunds_smoke():
    # synthetic left berm ridge
    xs = np.linspace(5, 30, 80)
    pts = np.column_stack(
        [
            np.repeat(xs, 5),
            np.tile(np.linspace(6.0, 8.0, 5), 80),
            np.tile(np.linspace(0.2, 1.5, 5), 80),
        ]
    ).astype(np.float32)
    dets, alerts = detect_bunds(pts)
    assert isinstance(dets, list)
    assert isinstance(alerts, list)


def test_vibration_alert():
    mon = VibrationMonitor()
    t = 0.0
    pose = (1.0, 2.0, 0.0)
    alert = None
    for i in range(200):
        t = i * 0.02
        # strong vertical shake
        az = 9.8 + 5.0 * np.sin(2 * np.pi * 8 * t)
        feat = mon.push(ImuSample(t=t, ax=0.0, ay=0.0, az=float(az)))
        if feat is not None:
            alert = mon.evaluate(feat, speed_mps=5.0, pose=pose) or alert
    assert alert is not None
    assert alert.type == 'excessive_vibration'
    d = alert.to_dict()
    assert d['schema'] == 'edge.alert.v1'
