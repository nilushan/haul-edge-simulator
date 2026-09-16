import struct

import numpy as np
import pytest

from edge_sensor_source.conversion import lidar_arrays, xyzi_bytes


def test_lidar_arrays_pads_short_channels_without_repeating_values():
    points, intensity = lidar_arrays({'xy': [[1, 2], [3, 4]], 'z': [5], 'i': [0.25]})
    np.testing.assert_allclose(points, [[1, 2, 5], [3, 4, 0]])
    np.testing.assert_allclose(intensity, [0.25, 1.0])


def test_lidar_arrays_rejects_unpaired_xy():
    with pytest.raises(ValueError, match='pairs'):
        lidar_arrays({'xy': [1, 2, 3]})


def test_xyzi_bytes_is_little_endian_and_aligned():
    data = xyzi_bytes(np.array([[1, 2, 3]], np.float32), np.array([4], np.float32))
    assert len(data) == 16
    assert struct.unpack('<ffff', data) == (1.0, 2.0, 3.0, 4.0)
