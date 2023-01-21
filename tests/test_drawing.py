from hypothesis import strategies as st, given
from shapely import geometry

from axi import Drawing

coords = st.tuples(
    st.floats(min_value=0, max_value=20, allow_nan=False, allow_subnormal=False, allow_infinity=False),
    st.floats(min_value=0, max_value=20, allow_nan=False, allow_subnormal=False, allow_infinity=False))
paths = st.lists(coords, min_size=2)
drawings = st.builds(Drawing, st.lists(paths, min_size=1))


@given(d_list=st.lists(drawings))
def test_combine(d_list: list[Drawing]):
    combined = Drawing.combine(d_list)
    assert len(combined.paths) == sum([len(d.paths) for d in d_list])
    for d in d_list:
        for path in d.paths:
            assert path in combined.paths


@given(d=drawings)
def test_points(d: Drawing):
    points = d.points
    assert len(points) == sum([len(path) for path in d.paths])
    for path in d.paths:
        for point in path:
            assert point in points


@given(d=drawings)
def test_convex_hull(d: Drawing):
    hull = d.convex_hull
    assert len(hull) >= 1
    for point in hull:
        assert point in d.points
    bounds = geometry.MultiPoint(hull).bounds
    assert d.bounds == bounds


@given(d=drawings)
def test_bounds(d: Drawing):
    x0, y0, x1, y1 = d.bounds
    assert 0 <= x0 <= x1 <= 20
    assert 0 <= y0 <= y1 <= 20
