from edge_viz.tick_buffer import TickBuffer


def test_detect_payload_on_tick():
    buf = TickBuffer()
    buf.start()
    buf.push_detect_cloud(
        'rocks',
        {'t': 1.0, 'n': 1, 'xy': [[3.0, 0.0]], 'z': [0.4], 'i': [1.0], 'label': [1], 'conf': [0.9]},
    )
    buf.push_alert(
        {
            'schema': 'edge.alert.v1',
            'event_id': 'e1',
            'type': 'rock',
            'severity': 'warn',
            't_ros': 1.0,
            'pose': {'x': 3.0, 'y': 0.0, 'z': 0.4},
        }
    )
    buf.push_vibe_features({'t': 1.1, 'rms_az': 1.7, 'peak_az': 3.2})
    tick = buf.live_tick()
    assert tick['detect']['clouds']['rocks']['n'] == 1
    assert tick['detect']['alerts'][0]['event_id'] == 'e1'
    assert tick['detect']['vibe']['rms_az'] == 1.7
    snap = buf.snapshot()
    assert snap['latest']['alerts_n'] == 1
