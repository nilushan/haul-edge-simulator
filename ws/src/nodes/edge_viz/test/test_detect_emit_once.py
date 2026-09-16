from edge_viz.tick_buffer import TickBuffer


ROCK_CLOUD = {
    't': 1.0,
    'n': 1,
    'xy': [[3.0, 0.0]],
    'z': [0.4],
    'i': [1.0],
    'label': [1],
    'conf': [0.9],
}


def test_detect_cloud_is_emit_once():
    """A detection cloud is consumed by the first live_tick and absent on the
    next, so stale body-frame clouds are not re-projected with advancing odom."""
    buf = TickBuffer()
    buf.start()
    buf.push_detect_cloud('rocks', ROCK_CLOUD)

    first = buf.live_tick()
    assert first['detect']['clouds']['rocks']['n'] == 1

    # No new rock cloud pushed: the second tick must NOT carry the stale cloud.
    second = buf.live_tick()
    assert 'rocks' not in second['detect']['clouds']

    # A freshly pushed cloud reappears on the next tick.
    buf.push_detect_cloud('rocks', ROCK_CLOUD)
    third = buf.live_tick()
    assert third['detect']['clouds']['rocks']['n'] == 1


def test_detect_cloud_clear_does_not_drop_alerts():
    buf = TickBuffer()
    buf.start()
    buf.push_detect_cloud('rocks', ROCK_CLOUD)
    buf.push_alert({'event_id': 'a1', 't_ros': 1.0, 'type': 'rock'})

    tick = buf.live_tick()
    assert tick['detect']['clouds']['rocks']['n'] == 1
    assert tick['detect']['alerts'][0]['event_id'] == 'a1'

    # Alerts persist (bounded deque); only per-frame clouds are emit-once.
    again = buf.live_tick()
    assert 'rocks' not in again['detect']['clouds']
    assert again['detect']['alerts'][0]['event_id'] == 'a1'