import numpy as np
import pytest

from edge_processor.cloud_ops import process_points


def test_process_points_filters_nonfinite_height_and_bounds_output():
    points = np.array(
        [[0, 0, -2], [0, 0, -1], [1, 0, 0], [2, 0, 1], [0, np.nan, 0]],
        dtype=np.float32,
    )
    result = process_points(points, z_min=-1.0, z_max=1.0, max_points=2)
    assert result.shape == (2, 3)
    assert np.all(np.isfinite(result))
    assert result[0, 2] == pytest.approx(-1.0)
    assert result[-1, 2] == pytest.approx(1.0)


def test_process_points_validates_configuration():
    with pytest.raises(ValueError):
        process_points(np.zeros((1, 3)), z_min=2.0, z_max=1.0, max_points=1)
    with pytest.raises(ValueError):
        process_points(np.zeros((1, 3)), z_min=0.0, z_max=1.0, max_points=0)
