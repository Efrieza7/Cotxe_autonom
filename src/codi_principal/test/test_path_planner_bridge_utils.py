import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from nodes.path_planning.bridge_utils import (  # noqa: E402
    build_unknown_cone_observations,
    extract_xy_path,
)


def test_build_unknown_cones_filters_and_maps():
    cons = [
        1.0,
        2.0,
        3.0,  # valid
        3.0,
        4.0,
        0.0,  # count filtered
        float("nan"),
        1.0,
        5.0,  # invalid x
    ]
    out = build_unknown_cone_observations(
        cons_data=cons,
        cone_types_count=5,
        unknown_index=0,
        min_cone_count=1,
    )
    assert len(out) == 5
    assert out[0].shape == (1, 2)
    assert np.allclose(out[0][0], np.array([1.0, 2.0]))
    for arr in out[1:]:
        assert arr.shape == (0, 2)


def test_extract_xy_path_from_fsd_shape():
    fsd_result = np.array(
        [
            [0.0, 10.0, 20.0, 0.1],
            [1.0, 11.0, 21.0, 0.2],
        ]
    )
    xy = extract_xy_path(fsd_result)
    assert xy is not None
    assert np.allclose(xy, np.array([[10.0, 20.0], [11.0, 21.0]]))


def test_extract_xy_path_rejects_bad_shapes():
    assert extract_xy_path(None) is None
    assert extract_xy_path(np.array([])) is None
    assert extract_xy_path(np.array([1.0, 2.0])) is None
