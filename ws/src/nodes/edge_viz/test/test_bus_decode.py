import struct
from types import SimpleNamespace

from edge_viz.bus_ingress import _cloud_to_lidar_dict


def _field(name, offset):
    return SimpleNamespace(name=name, offset=offset, datatype=7, count=1)


def test_cloud_decode_honors_big_endian_and_organized_row_padding():
    data = bytearray(40)
    struct.pack_into('>ffff', data, 0, 1.0, 2.0, 3.0, 0.25)
    struct.pack_into('>ffff', data, 20, 4.0, 5.0, 6.0, 0.75)
    message = SimpleNamespace(
        fields=[_field('x', 0), _field('y', 4), _field('z', 8), _field('intensity', 12)],
        point_step=16,
        row_step=20,
        width=1,
        height=2,
        is_bigendian=True,
        data=bytes(data),
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=500_000_000)),
    )
    payload = _cloud_to_lidar_dict(message)
    assert payload['t'] == 1.5
    assert payload['n'] == 2
    assert payload['xy'] == [[1.0, 2.0], [4.0, 5.0]]
    assert payload['z'] == [3.0, 6.0]
    assert payload['i'] == [0.25, 0.75]
