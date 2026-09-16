"""On-disk replayable sensor stream format (JSONL + manifest)."""

from __future__ import annotations

import json
import math
import os
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional


STREAM_VERSION = 1


@dataclass
class StreamManifest:
    version: int = STREAM_VERSION
    stream_id: str = ''
    map_id: str = ''
    map_title: str = ''
    duration_s: float = 60.0
    speed_mps: float = 8.0
    imu_hz: float = 50.0
    gnss_hz: float = 5.0
    lidar_hz: float = 10.0
    vehicle_hz: float = 50.0
    lidar_max_points: int = 20000
    description: str = ''
    files: Dict[str, str] = field(
        default_factory=lambda: {
            'odom': 'odom.jsonl',
            'imu': 'imu.jsonl',
            'gnss': 'gnss.jsonl',
            'lidar': 'lidar.jsonl',
        }
    )

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> 'StreamManifest':
        if not isinstance(d, dict):
            raise TypeError('stream manifest must be a JSON object')
        version = d.get('version', STREAM_VERSION)
        if version != STREAM_VERSION:
            raise ValueError(f'unsupported stream version {version!r}; expected {STREAM_VERSION}')
        if 'files' in d and not isinstance(d['files'], dict):
            raise ValueError('stream manifest files must be an object mapping kinds to paths')
        known = {f.name for f in cls.__dataclass_fields__.values()}  # type: ignore[attr-defined]
        payload = {k: v for k, v in d.items() if k in known}
        manifest = cls(**payload)
        for name in ('duration_s', 'imu_hz', 'gnss_hz', 'lidar_hz', 'vehicle_hz'):
            value = float(getattr(manifest, name))
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'manifest {name} must be a positive finite value')
        return manifest

    def save(self, root: Path) -> None:
        root.mkdir(parents=True, exist_ok=True)
        (root / 'manifest.json').write_text(json.dumps(self.to_dict(), indent=2) + '\n')

    @classmethod
    def load(cls, root: Path) -> 'StreamManifest':
        path = root / 'manifest.json'
        data = json.loads(path.read_text())
        return cls.from_dict(data)


class StreamWriter:
    """Record a multi-rate sensor stream to a directory."""

    def __init__(self, root: Path | str, manifest: StreamManifest) -> None:
        self.root = Path(root)
        self.manifest = manifest
        self.root.mkdir(parents=True, exist_ok=True)
        self.manifest.save(self.root)
        self._files = {
            'odom': (self.root / manifest.files['odom']).open('w', encoding='utf-8'),
            'imu': (self.root / manifest.files['imu']).open('w', encoding='utf-8'),
            'gnss': (self.root / manifest.files['gnss']).open('w', encoding='utf-8'),
            'lidar': (self.root / manifest.files['lidar']).open('w', encoding='utf-8'),
        }

    def write(self, kind: str, sample: Dict[str, Any]) -> None:
        f = self._files[kind]
        f.write(json.dumps(sample, separators=(',', ':')) + '\n')

    def close(self) -> None:
        for f in self._files.values():
            f.close()

    def __enter__(self) -> 'StreamWriter':
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()


def _read_jsonl(path: Path) -> List[Dict[str, Any]]:
    if not path.is_file():
        return []
    out: List[Dict[str, Any]] = []
    with path.open('r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            sample = json.loads(line)
            if not isinstance(sample, dict):
                raise ValueError(f'stream sample in {path} must be a JSON object')
            t = float(sample.get('t', 0.0))
            if not math.isfinite(t):
                raise ValueError(f'stream sample in {path} has a non-finite timestamp')
            out.append(sample)
    out.sort(key=lambda sample: float(sample.get('t', 0.0)))
    return out


class StreamReader:
    """Load a recorded stream and expose time-ordered samples for replay."""

    def __init__(self, root: Path | str) -> None:
        self.root = Path(root)
        if not (self.root / 'manifest.json').is_file():
            raise FileNotFoundError(f'no stream manifest at {self.root}')
        self.manifest = StreamManifest.load(self.root)
        files = self.manifest.files
        self.odom = _read_jsonl(self.root / files.get('odom', 'odom.jsonl'))
        self.imu = _read_jsonl(self.root / files.get('imu', 'imu.jsonl'))
        self.gnss = _read_jsonl(self.root / files.get('gnss', 'gnss.jsonl'))
        self.lidar = _read_jsonl(self.root / files.get('lidar', 'lidar.jsonl'))
        self.duration_s = float(self.manifest.duration_s)
        if self.odom:
            self.duration_s = max(self.duration_s, float(self.odom[-1].get('t', 0.0)))
        if self.imu:
            self.duration_s = max(self.duration_s, float(self.imu[-1].get('t', 0.0)))
        if self.gnss:
            self.duration_s = max(self.duration_s, float(self.gnss[-1].get('t', 0.0)))
        if self.lidar:
            self.duration_s = max(self.duration_s, float(self.lidar[-1].get('t', 0.0)))

    def summary(self) -> Dict[str, Any]:
        return {
            'path': str(self.root.resolve()),
            'stream_id': self.manifest.stream_id or self.root.name,
            'map_id': self.manifest.map_id,
            'map_title': self.manifest.map_title,
            'duration_s': self.duration_s,
            'counts': {
                'odom': len(self.odom),
                'imu': len(self.imu),
                'gnss': len(self.gnss),
                'lidar': len(self.lidar),
            },
            'rates': {
                'imu_hz': self.manifest.imu_hz,
                'gnss_hz': self.manifest.gnss_hz,
                'lidar_hz': self.manifest.lidar_hz,
                'vehicle_hz': self.manifest.vehicle_hz,
            },
        }


def discover_streams(root: Path | str) -> List[Dict[str, Any]]:
    """Find stream directories under root (each with manifest.json)."""
    base = Path(root)
    if not base.is_dir():
        return []
    found: List[Dict[str, Any]] = []
    for child in sorted(base.iterdir()):
        if not child.is_dir():
            continue
        if not (child / 'manifest.json').is_file():
            continue
        try:
            man = StreamManifest.load(child)
        except (OSError, json.JSONDecodeError, TypeError, ValueError):
            continue
        found.append(
            {
                'id': man.stream_id or child.name,
                'path': str(child.resolve()),
                'map_id': man.map_id,
                'map_title': man.map_title,
                'duration_s': man.duration_s,
                'description': man.description,
            }
        )
    return found


def default_streams_root(project_root: Optional[Path] = None) -> Path:
    if project_root is None:
        configured = os.environ.get('HAUL_EDGE_PROJECT_ROOT')
        if configured:
            project_root = Path(configured).expanduser()
        else:
            here = Path(__file__).resolve()
            project_root = next(
                (
                    parent
                    for parent in (here.parent, *here.parents)
                    if (parent / 'ws' / 'src').is_dir() and (parent / 'data').is_dir()
                ),
                Path.cwd(),
            )
    return project_root / 'data' / 'streams'
