"""Named haul-map presets. Each map is a deterministic world configuration."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Dict, List, Sequence, Tuple

from edge_sim.models.world import HaulWorld, PathProfile


@dataclass(frozen=True)
class MapSpec:
    """Declarative map identity + terrain knobs."""

    id: str
    title: str
    description: str
    seed: int = 19
    road_half_width_m: float = 6.5
    berm_height_m: float = 1.65
    berm_width_m: float = 2.0
    n_rocks: int = 48
    rock_radius: Tuple[float, float] = (0.3, 1.15)
    # Under-built / missing berm sections seeded along the route.
    n_bund_defects: int = 4
    speed_mps: float = 8.0
    # Centerline: list of (amplitude_m, frequency_1/m, phase_rad)
    y_terms: Tuple[Tuple[float, float, float], ...] = (
        (55.0, 0.0065, 0.0),
        (22.0, 0.0028, 0.7),
        (8.0, 0.014, 1.2),
    )
    x_wiggle_amp: float = 12.0
    x_wiggle_freq: float = 0.004
    # Grades: (start_s_m, rise_m, ramp_length_m)
    grades: Tuple[Tuple[float, float, float], ...] = (
        (60.0, 9.0, 140.0),
        (260.0, -6.5, 70.0),
        (380.0, 4.0, 90.0),
        (520.0, -2.5, 50.0),
    )
    undulation: Tuple[Tuple[float, float, float], ...] = (
        (1.4, 0.018, 0.0),
        (0.7, 0.04, 0.8),
    )

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        return d

    def path_profile(self) -> PathProfile:
        return PathProfile(
            y_terms=self.y_terms,
            x_wiggle_amp=self.x_wiggle_amp,
            x_wiggle_freq=self.x_wiggle_freq,
            grades=self.grades,
            undulation=self.undulation,
        )

    def build_world(self) -> HaulWorld:
        return HaulWorld(
            road_half_width_m=self.road_half_width_m,
            berm_height_m=self.berm_height_m,
            berm_width_m=self.berm_width_m,
            seed=self.seed,
            n_rocks=self.n_rocks,
            rock_radius=self.rock_radius,
            path=self.path_profile(),
            n_bund_defects=self.n_bund_defects,
        )


# --- Preset catalog ---------------------------------------------------------

MAPS: Dict[str, MapSpec] = {
    'haul_corridor': MapSpec(
        id='haul_corridor',
        title='Haul corridor',
        description='Default snaking corridor with berms, grades, and mixed rocks.',
        seed=19,
    ),
    'tight_switchbacks': MapSpec(
        id='tight_switchbacks',
        title='Tight switchbacks',
        description='Narrow road, sharp lateral wiggles, tall bunds — mine switchback feel.',
        seed=41,
        road_half_width_m=4.8,
        berm_height_m=2.2,
        berm_width_m=1.6,
        n_rocks=36,
        rock_radius=(0.25, 0.9),
        speed_mps=6.0,
        y_terms=(
            (38.0, 0.012, 0.2),
            (18.0, 0.021, 1.1),
            (10.0, 0.035, 0.4),
        ),
        x_wiggle_amp=6.0,
        x_wiggle_freq=0.009,
        grades=(
            (40.0, 12.0, 90.0),
            (180.0, -10.0, 60.0),
            (300.0, 7.0, 80.0),
            (430.0, -5.0, 55.0),
        ),
        undulation=(
            (1.0, 0.03, 0.0),
            (0.5, 0.06, 1.0),
        ),
    ),
    'open_pit_bench': MapSpec(
        id='open_pit_bench',
        title='Open-pit bench',
        description='Wide bench road, long gentle curve, sparse rocks, big elevation steps.',
        seed=77,
        road_half_width_m=9.0,
        berm_height_m=1.2,
        berm_width_m=2.8,
        n_rocks=22,
        rock_radius=(0.4, 1.4),
        speed_mps=9.0,
        y_terms=(
            (90.0, 0.0035, 0.0),
            (25.0, 0.0018, 0.9),
            (5.0, 0.01, 0.3),
        ),
        x_wiggle_amp=20.0,
        x_wiggle_freq=0.0025,
        grades=(
            (80.0, 18.0, 200.0),
            (320.0, -14.0, 120.0),
            (500.0, 8.0, 150.0),
        ),
        undulation=(
            (0.8, 0.012, 0.0),
            (0.35, 0.028, 0.5),
        ),
    ),
    'rocky_descent': MapSpec(
        id='rocky_descent',
        title='Rocky descent',
        description='Steep downhill haul with dense rock field and rough shoulders.',
        seed=103,
        road_half_width_m=6.0,
        berm_height_m=1.9,
        berm_width_m=2.2,
        n_rocks=72,
        rock_radius=(0.35, 1.35),
        speed_mps=7.0,
        y_terms=(
            (40.0, 0.007, 0.5),
            (15.0, 0.015, 1.4),
            (6.0, 0.022, 0.2),
        ),
        x_wiggle_amp=10.0,
        x_wiggle_freq=0.0055,
        grades=(
            (20.0, 4.0, 40.0),
            (90.0, -16.0, 180.0),
            (320.0, -6.0, 90.0),
            (450.0, 5.0, 70.0),
        ),
        undulation=(
            (2.0, 0.025, 0.0),
            (1.1, 0.05, 0.7),
        ),
    ),
}

DEFAULT_MAP_ID = 'haul_corridor'
DEFAULT_PLAYLIST: Tuple[str, ...] = (
    'haul_corridor',
    'tight_switchbacks',
    'open_pit_bench',
    'rocky_descent',
)


def list_maps() -> List[Dict[str, Any]]:
    return [
        {
            'id': m.id,
            'title': m.title,
            'description': m.description,
            'speed_mps': m.speed_mps,
            'seed': m.seed,
        }
        for m in MAPS.values()
    ]


def get_map(map_id: str) -> MapSpec:
    key = (map_id or DEFAULT_MAP_ID).strip()
    if key not in MAPS:
        known = ', '.join(MAPS)
        raise KeyError(f'unknown map {map_id!r}; choose one of: {known}')
    return MAPS[key]


def resolve_playlist(maps: Sequence[str] | None) -> List[MapSpec]:
    ids = list(maps) if maps else list(DEFAULT_PLAYLIST)
    if not ids:
        ids = [DEFAULT_MAP_ID]
    return [get_map(i) for i in ids]
