"""Thread-safe SQLite persistence for edge alerts (ROS-independent)."""

from __future__ import annotations

import json
import math
import sqlite3
import threading
import time
import uuid
from pathlib import Path
from typing import Any, Dict, Optional


def normalize_alert(payload: Any, now_s: Optional[float] = None) -> tuple[str, float, str, str, str]:
    """Validate an alert payload and return normalized storage fields."""
    if not isinstance(payload, dict):
        raise ValueError('alert payload must be a JSON object')
    now_s = time.time() if now_s is None else float(now_s)
    event_id = str(payload.get('event_id') or uuid.uuid4())
    timestamp = payload.get('t_ros')
    if timestamp is None:
        timestamp = payload.get('t_vehicle')
    if timestamp is None:
        timestamp = now_s
    timestamp = float(timestamp)
    if not math.isfinite(timestamp):
        raise ValueError('alert timestamp must be finite')
    event_type = str(payload.get('type') or 'unknown')
    severity = str(payload.get('severity') or '')
    encoded = json.dumps(payload, allow_nan=False, separators=(',', ':'))
    return event_id, timestamp, event_type, severity, encoded


class EventStore:
    """Bounded, thread-safe local event database."""

    def __init__(self, path: Path | str) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._conn = sqlite3.connect(str(self.path), check_same_thread=False)
        with self._lock:
            self._conn.execute(
                """
                CREATE TABLE IF NOT EXISTS events (
                  id TEXT PRIMARY KEY,
                  t REAL NOT NULL,
                  type TEXT NOT NULL,
                  severity TEXT,
                  json TEXT NOT NULL,
                  created_at REAL NOT NULL
                )
                """
            )
            self._conn.execute('CREATE INDEX IF NOT EXISTS idx_events_t ON events(t)')
            self._conn.execute('CREATE INDEX IF NOT EXISTS idx_events_created ON events(created_at)')
            self._conn.commit()

    def insert(self, payload: Dict[str, Any], now_s: Optional[float] = None) -> bool:
        now_s = time.time() if now_s is None else float(now_s)
        event_id, timestamp, event_type, severity, encoded = normalize_alert(payload, now_s)
        with self._lock:
            cursor = self._conn.execute(
                'INSERT OR IGNORE INTO events (id, t, type, severity, json, created_at) VALUES (?,?,?,?,?,?)',
                (event_id, timestamp, event_type, severity, encoded, now_s),
            )
            self._conn.commit()
            return cursor.rowcount > 0

    def count(self) -> int:
        with self._lock:
            row = self._conn.execute('SELECT COUNT(*) FROM events').fetchone()
            return int(row[0] if row else 0)

    def prune(self, *, max_rows: int, max_age_s: float, now_s: Optional[float] = None) -> int:
        if max_rows < 1 or max_age_s <= 0:
            raise ValueError('max_rows and max_age_s must be positive')
        now_s = time.time() if now_s is None else float(now_s)
        removed = 0
        with self._lock:
            cursor = self._conn.execute(
                'DELETE FROM events WHERE created_at < ?',
                (now_s - max_age_s,),
            )
            removed += max(cursor.rowcount, 0)
            cursor = self._conn.execute(
                """
                DELETE FROM events
                WHERE id IN (
                    SELECT id FROM events
                    ORDER BY created_at DESC, rowid DESC
                    LIMIT -1 OFFSET ?
                )
                """,
                (max_rows,),
            )
            removed += max(cursor.rowcount, 0)
            self._conn.commit()
        return removed

    def close(self) -> None:
        with self._lock:
            self._conn.close()
