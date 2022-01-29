from math import hypot
from typing import Optional

import numpy as np
from pyhull.convex_hull import ConvexHull
from scipy.spatial import KDTree
from shapely import geometry

from .spatial import Index

Point = tuple[float, float]
Path = list[Point]


def load_paths(filename: str) -> list[Path]:
    paths = []
    with open(filename) as fp:
        for line in fp:
            points = filter(None, line.strip().split(";"))
            if not points:
                continue
            path = [tuple(map(float, x.split(","))) for x in points]
            paths.append(path)
    return paths


def path_length(points: Path) -> float:
    result = 0
    for (x1, y1), (x2, y2) in zip(points, points[1:]):
        result += hypot(x2 - x1, y2 - y1)
    return result


def paths_length(paths: list[Path]) -> float:
    return sum([path_length(path) for path in paths], 0)


def simplify_path(points: Path, tolerance: float) -> Path:
    if len(points) < 2:
        return points
    line = geometry.LineString(points)
    line = line.simplify(tolerance, preserve_topology=False)
    return list(line.coords)


def simplify_paths(paths: list[Path], tolerance: float) -> list[Path]:
    return [simplify_path(x, tolerance) for x in paths]


def sort_paths(paths: list[Path], reversible: bool = True) -> list[Path]:
    if len(paths) <= 1:
        return paths
    first = paths[0]
    paths.remove(first)
    result = [first]
    points = []
    for path in paths:
        x1, y1 = path[0]
        x2, y2 = path[-1]
        points.append((x1, y1, path, False))
        if reversible:
            points.append((x2, y2, path, True))
    index = Index(points)
    while index.size > 0:
        x, y, path, reverse = index.nearest(result[-1][-1])
        x1, y1 = path[0]
        x2, y2 = path[-1]
        index.remove((x1, y1, path, False))
        if reversible:
            index.remove((x2, y2, path, True))
        if reverse:
            result.append(list(reversed(path)))
        else:
            result.append(path)
    return result


class LineIndex:
    def __init__(self, lines):
        self.lines = [line for line in lines if len(line) > 0]
        self.index = None
        self.r_index = None
        self.reindex()

    def reindex(self):
        self.index = KDTree(np.array([line[0] for line in self.lines]))
        self.r_index = KDTree(np.array([line[-1] for line in self.lines]))

    def find_nearest_within(
        self, p: Point, tolerance: float
    ) -> tuple[Optional[int], bool]:
        dist, idx = self.index.query(p, distance_upper_bound=tolerance)
        if idx < len(self.lines):
            return idx, False
        dist, idx = self.r_index.query(p, distance_upper_bound=tolerance)
        if idx < len(self.lines):
            return idx, True
        return None, False

    def pop(self, idx: int) -> Path:
        out = self.lines.pop(idx)
        return out

    def __len__(self):
        return len(self.lines)


def join_paths(paths: list[Path], tolerance: float) -> list[Path]:
    paths = [path for path in paths if len(path) > 0]
    if len(paths) < 2:
        return paths
    line_index = LineIndex(paths)
    out = []
    while len(line_index) > 1:
        path = line_index.pop(0)
        line_index.reindex()
        while True:
            idx, reverse = line_index.find_nearest_within(path[-1], tolerance)
            if idx is None:
                idx, reverse = line_index.find_nearest_within(path[0], tolerance)
                if idx is None:
                    break
                path = path[::-1]
            extension = line_index.pop(idx)
            if reverse:
                extension = extension[::-1]
            path.extend(extension[1:])
            if len(line_index) >= 1:
                line_index.reindex()
            else:
                break
        out.append(path)
    return out


def crop_interpolate(
    x1: float,
    y1: float,
    x2: float,
    y2: float,
    ax: float,
    ay: float,
    bx: float,
    by: float,
) -> tuple[float, float]:
    dx = bx - ax
    dy = by - ay
    t1 = (x1 - ax) / dx if dx else -1
    t2 = (y1 - ay) / dy if dy else -1
    t3 = (x2 - ax) / dx if dx else -1
    t4 = (y2 - ay) / dy if dy else -1
    ts = [t1, t2, t3, t4]
    ts = [t for t in ts if 0 <= t <= 1]
    t = min(ts)
    x = ax + (bx - ax) * t
    y = ay + (by - ay) * t
    return x, y


def crop_path(path: Path, x1: float, y1: float, x2: float, y2: float) -> Path:
    e = 1e-9
    result = []
    buf = []
    previous_point = None
    previous_inside = False
    for x, y in path:
        inside = x1 - e <= x <= x2 + e and y1 - e <= y <= y2 + e
        if inside:
            if not previous_inside and previous_point:
                px, py = previous_point
                ix, iy = crop_interpolate(x1, y1, x2, y2, x, y, px, py)
                buf.append((ix, iy))
            buf.append((x, y))
        else:
            if previous_inside and previous_point:
                px, py = previous_point
                ix, iy = crop_interpolate(x1, y1, x2, y2, x, y, px, py)
                buf.append((ix, iy))
                result.append(buf)
                buf = []
        previous_point = (x, y)
        previous_inside = inside
    if buf:
        result.append(buf)
    return result


def crop_paths(
    paths: list[Path], x1: float, y1: float, x2: float, y2: float
) -> list[Path]:
    return [crop_path(path, x1, y1, x2, y2) for path in paths]


def convex_hull(points: list[Point]) -> list[Point]:
    hull = ConvexHull(points)
    vertices = set(i for v in hull.vertices for i in v)
    return [hull.points[i] for i in vertices]


def quadratic_path(
    x0: float, y0: float, x1: float, y1: float, x2: float, y2: float
) -> Path:
    n = int(hypot(x1 - x0, y1 - y0) + hypot(x2 - x1, y2 - y1))
    n = max(n, 4)
    points = []
    m = 1 / float(n - 1)
    for i in range(n):
        t = i * m
        u = 1 - t
        a = u * u
        b = 2 * u * t
        c = t * t
        x = a * x0 + b * x1 + c * x2
        y = a * y0 + b * y1 + c * y2
        points.append((x, y))
    return points


def expand_quadratics(path):
    result = []
    previous = (0, 0)
    for point in path:
        if len(point) == 2:
            result.append(point)
            previous = point
        elif len(point) == 4:
            x0, y0 = previous
            x1, y1, x2, y2 = point
            result.extend(quadratic_path(x0, y0, x1, y1, x2, y2))
            previous = (x2, y2)
        else:
            raise Exception("invalid point: %r" % point)
    return result


def paths_to_shapely(paths: list[Path]) -> geometry.MultiLineString:
    # TODO: Polygons for closed paths?
    return geometry.MultiLineString(paths)


def shapely_to_paths(g) -> list[Path]:
    if isinstance(g, geometry.Point):
        return []
    elif isinstance(g, geometry.LineString):
        return [list(g.coords)]
    elif isinstance(
        g,
        (
            geometry.MultiPoint,
            geometry.MultiLineString,
            geometry.MultiPolygon,
            geometry.collection.GeometryCollection,
        ),
    ):
        paths = []
        for x in g:
            paths.extend(shapely_to_paths(x))
        return paths
    elif isinstance(g, geometry.Polygon):
        paths = [list(g.exterior.coords)]
        for interior in g.interiors:
            paths.extend(shapely_to_paths(interior))
        return paths
    else:
        raise Exception("unhandled shapely geometry: %s" % type(g))
