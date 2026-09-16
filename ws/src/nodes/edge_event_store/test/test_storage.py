import json

import pytest

from edge_event_store.storage import EventStore, normalize_alert


def test_normalize_preserves_zero_timestamp_and_generates_id():
    event_id, timestamp, kind, severity, encoded = normalize_alert({'t_ros': 0.0, 'type': 'rock'})
    assert event_id
    assert timestamp == 0.0
    assert kind == 'rock'
    assert severity == ''
    assert json.loads(encoded)['t_ros'] == 0.0


def test_store_rejects_non_object_and_deduplicates(tmp_path):
    store = EventStore(tmp_path / 'events.sqlite')
    with pytest.raises(ValueError):
        store.insert(['not', 'an', 'object'])
    payload = {'event_id': 'same', 't_ros': 1.0, 'type': 'rock'}
    assert store.insert(payload, now_s=10.0)
    assert not store.insert(payload, now_s=11.0)
    assert store.count() == 1
    store.close()


def test_store_prunes_by_age_and_row_limit(tmp_path):
    store = EventStore(tmp_path / 'events.sqlite')
    for index in range(5):
        store.insert({'event_id': str(index), 't_ros': float(index)}, now_s=float(index))
    removed = store.prune(max_rows=2, max_age_s=100.0, now_s=5.0)
    assert removed == 3
    assert store.count() == 2
    store.insert({'event_id': 'old', 't_ros': 0.0}, now_s=0.0)
    assert store.prune(max_rows=10, max_age_s=2.0, now_s=5.0) >= 1
    store.close()
