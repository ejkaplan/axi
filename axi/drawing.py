from __future__ import division, annotations

from copy import deepcopy
from dataclasses import dataclass
from math import sin, cos, radians, hypot
from typing import Optional

import matplotlib
import numpy as np
from shapely.affinity import rotate
from shapely.errors import TopologicalError
from shapely.geometry import MultiLineString, Polygon, Point, LineString, MultiPolygon, \
    GeometryCollection
from shapely.ops import split

from .paths import (
    simplify_paths,
    sort_paths,
    join_paths,
    convex_hull,
    expand_quadratics,
    paths_length,
    Path,
    reloop_paths,
)

matplotlib.use("TkAgg")

V3_SIZE = (12, 8.5)
V3_BOUNDS = (0, 0, 12, 8.5)

A3_SIZE = (16.93, 11.69)
A3_BOUNDS = (0, 0, 16.93, 11.69)


@dataclass
class Drawing(object):
    def __init__(self, paths: Optional[list[Path]] = None):
        self.paths = paths or []
        self._bounds = None
        self._length = None
        self._down_length = None
        self._hull = None

    def __eq__(self, other: Drawing):
        return self.paths == other.paths

    def __hash__(self):
        return hash(tuple(self.paths))

    def copy(self) -> Drawing:
        return Drawing(deepcopy(self.paths))

    def dirty(self) -> None:
        self._bounds = None
        self._length = None
        self._down_length = None
        self._hull = None

    @staticmethod
    def combine(drawings: list[Drawing]) -> Drawing:
        all_paths = [path for d in drawings for path in d.paths]
        return Drawing(all_paths)

    @classmethod
    def loads(cls, data: str) -> Drawing:
        paths = []
        for line in data.split("\n"):
            line = line.strip()
            if line.startswith("#"):
                continue
            path = line.split()
            path = [tuple(map(float, x.split(","))) for x in path]
            path = expand_quadratics(path)
            if path:
                paths.append(path)
        return cls(paths)

    @classmethod
    def load(cls, filename: str) -> Drawing:
        with open(filename, "r") as fp:
            return cls.loads(fp.read())

    def dumps(self) -> str:
        lines = []
        for path in self.paths:
            lines.append(" ".join("%f,%f" % (x, y) for x, y in path))
        return "\n".join(lines)

    def dump(self, filename: str) -> None:
        with open(filename, "w") as fp:
            fp.write(self.dumps())

    def dumps_svg(self, scale: float = 96) -> str:
        lines = []
        w = (self.width + 2) * scale
        h = (self.height + 2) * scale
        lines.append(
            '<svg xmlns="http://www.w3.org/2000/svg" version="1.1" width="%g" height="%g">'
            % (w, h)
        )
        lines.append('<g transform="scale(%g) translate(1 1)">' % scale)
        for path in self.paths:
            p = []
            c = "M"
            for x, y in path:
                p.append("%s%g %g" % (c, x, y))
                c = "L"
            d = " ".join(p)
            lines.append(
                '<path d="%s" fill="none" stroke="black" stroke-width="0.01" stroke-linecap="round" stroke-linejoin="round" />'
                % d
            )
        lines.append("</g>")
        lines.append("</svg>")
        return "\n".join(lines)

    def dump_svg(self, filename: str) -> None:
        with open(filename, "w") as fp:
            fp.write(self.dumps_svg())

    def to_shapely(self) -> MultiLineString:
        return MultiLineString(self.paths)

    @staticmethod
    def from_shapely(shape) -> Drawing:
        if isinstance(shape, LineString):
            return Drawing([list(shape.coords)])
        elif isinstance(shape, Polygon):
            return Drawing(
                [list(shape.exterior.coords)] + [list(hole.coords) for hole in
                                                 shape.interiors])
        elif isinstance(shape, MultiLineString | MultiPolygon | GeometryCollection):
            out = Drawing()
            for elem in shape.geoms:
                out = out.add(Drawing.from_shapely(elem))
            return out
        else:
            return Drawing()

    @property
    def points(self) -> list[Point]:
        return [(x, y) for path in self.paths for x, y in path]

    @property
    def convex_hull(self) -> Path:
        if self._hull is None:
            self._hull = convex_hull(self.points)
        return self._hull

    @property
    def bounds(self) -> tuple[float, float, float, float]:
        if self._bounds is None:
            points = self.points
            if points:
                x1 = min(x for x, y in points)
                x2 = max(x for x, y in points)
                y1 = min(y for x, y in points)
                y2 = max(y for x, y in points)
            else:
                x1 = x2 = y1 = y2 = 0
            self._bounds = (x1, y1, x2, y2)
        return self._bounds

    @property
    def length(self) -> float:
        if self._length is None:
            length = self.down_length
            for p0, p1 in zip(self.paths, self.paths[1:]):
                x0, y0 = p0[-1]
                x1, y1 = p1[0]
                length += hypot(x1 - x0, y1 - y0)
            self._length = length
        return self._length

    @property
    def up_length(self) -> float:
        return self.length - self.down_length

    @property
    def down_length(self) -> float:
        if self._down_length is None:
            self._down_length = paths_length(self.paths)
        return self._down_length

    @property
    def width(self) -> float:
        x1, y1, x2, y2 = self.bounds
        return x2 - x1

    @property
    def height(self) -> float:
        x1, y1, x2, y2 = self.bounds
        return y2 - y1

    @property
    def size(self) -> tuple[float, float]:
        return self.width, self.height

    @property
    def all_paths(self) -> list[Path]:
        result = []
        position = (0, 0)
        for path in self.paths:
            result.append([position, path[0]])
            result.append(path)
            position = path[-1]
        result.append([position, (0, 0)])
        return result

    def simplify_paths(self, tolerance: float) -> Drawing:
        return Drawing(simplify_paths(self.paths, tolerance))

    def sort_paths(self, reversible: bool = True) -> Drawing:
        return Drawing(sort_paths(self.paths, reversible))

    def join_paths(self, tolerance: float) -> Drawing:
        return Drawing(join_paths(self.paths, tolerance))

    def reloop_paths(
            self, reverse: bool = False, rng: Optional[np.random.Generator] = None
    ) -> Drawing:
        return Drawing(reloop_paths(self.paths, reverse, rng))

    def crop_to_rectangle(self, x1: float, y1: float, x2: float, y2: float) -> Drawing:
        boundary = Polygon([(x1, y1), (x2, y1), (x2, y2), (x1, y2)])
        return self.crop_to_boundary(boundary)

    def crop_to_boundary(self, boundary: Polygon | MultiPolygon) -> Drawing:
        multiline = self.to_shapely()
        splits = split(multiline, boundary)
        lines = []
        for line in splits.geoms:
            try:
                if len(line.coords) >= 2 and boundary.contains(line):
                    lines.append(line)
            except TopologicalError:
                pass
        return Drawing([list(line.coords) for line in lines])

    def add(self, drawing: Drawing) -> Drawing:
        return Drawing(self.paths + drawing.paths)

    def transform(self, func) -> Drawing:
        return Drawing([[func(x, y) for x, y in path] for path in self.paths])

    def translate(self, dx: float, dy: float) -> Drawing:
        def func(x, y):
            return x + dx, y + dy

        return self.transform(func)

    def scale(self, sx: float, sy: Optional[float] = None) -> Drawing:
        if sy is None:
            sy = sx

        def func(x, y):
            return x * sx, y * sy

        return self.transform(func)

    def rotate(
            self, angle: float, anchor: tuple[float, float] = (0, 0),
            degrees: bool = False
    ) -> Drawing:
        d = self.translate(-anchor[0], -anchor[1])
        if degrees:
            angle = radians(angle)
        c = cos(angle)
        s = sin(angle)

        def func(x, y):
            return x * c - y * s, y * c + x * s

        d = d.transform(func)
        d = d.translate(anchor[0], anchor[1])
        return d

    def move(self, x: float, y: float, ax: float, ay: float) -> Drawing:
        x1, y1, x2, y2 = self.bounds
        dx = x1 + (x2 - x1) * ax - x
        dy = y1 + (y2 - y1) * ay - y
        return self.translate(-dx, -dy)

    def origin(self) -> Drawing:
        return self.move(0, 0, 0, 0)

    def center(self, width: float, height: float) -> Drawing:
        return self.move(width / 2, height / 2, 0.5, 0.5)

    def rotate_to_fit(
            self, width: float, height: float, step: float = 0.05, degrees: bool = False
    ) -> Optional[Drawing]:
        if degrees:
            step = radians(step)
        for angle in np.arange(0, np.pi, step):
            drawing = self.rotate(angle)
            if drawing.width <= width and drawing.height <= height:
                return drawing.center(width, height)
        return None

    def scale_to_fit_height(self, height: float, padding: float = 0) -> Drawing:
        return self.scale_to_fit(float("inf"), height, padding)

    def scale_to_fit_width(self, width: float, padding: float = 0) -> Drawing:
        return self.scale_to_fit(width, float("inf"), padding)

    def scale_to_fit(self, width: float, height: float, padding: float = 0) -> Drawing:
        if self.width == self.height == 0:
            return self
        width -= padding * 2
        height -= padding * 2
        if self.width == 0:
            scale = height / self.height
        elif self.height == 0:
            scale = width / self.width
        else:
            scale = min(width / self.width, height / self.height)
        return self.scale(scale, scale).center(width, height)

    @staticmethod
    def multi_scale_to_fit(
            drawings: list[Drawing], width: float, height: float, padding: float = 0
    ) -> list[Drawing]:
        combined = Drawing.combine(drawings)
        combined_scaled = combined.scale_to_fit(width, height, padding)
        if combined.width == combined.height == 0:
            return drawings
        scale_x = combined_scaled.width / combined.width
        scale_y = combined_scaled.height / combined.height
        combined_scaled_no_offset = combined.scale(scale_x, scale_y)
        offset_x = (
                combined_scaled.bounds[0] - combined_scaled_no_offset.bounds[
            0] + padding
        )
        offset_y = (
                combined_scaled.bounds[1] - combined_scaled_no_offset.bounds[
            1] + padding
        )
        return [
            d.scale(scale_x, scale_y).translate(offset_x, offset_y) for d in drawings
        ]

    def rotate_and_scale_to_fit(
            self,
            width: float,
            height: float,
            padding: float = 0,
            step=0.01,
            degrees: bool = False,
    ) -> Drawing:
        if degrees:
            step = radians(step)
        values = []
        width -= padding * 2
        height -= padding * 2
        hull = Drawing([self.convex_hull])
        for angle in np.arange(0, np.pi, step):
            d = hull.rotate(angle)
            scale = min(width / d.width, height / d.height)
            values.append((scale, angle))
        scale, angle = max(values)
        return self.rotate(angle).scale(scale, scale).center(width, height)

    def remove_paths_outside(self, width: float, height: float) -> Drawing:
        e = 1e-8
        paths = []
        for path in self.paths:
            ok = True
            for x, y in path:
                if x < -e or y < -e or x > width + e or y > height + e:
                    ok = False
                    break
            if ok:
                paths.append(path)
        return Drawing(paths)

    def repeat(self, n: int = 2) -> Drawing:
        paths = []
        for path in self.paths:
            tmp = path
            new_path = path
            for _ in range(n - 1):
                tmp = tmp[::-1]
                new_path += tmp[1:]
            paths.append(new_path)
        return Drawing(paths)

    @staticmethod
    def shade(polygon: Polygon, angle: float, spacing: float) -> Drawing:
        out = Drawing()
        centroid = polygon.centroid
        polygon = rotate(polygon, -angle, use_radians=True, origin=centroid)
        x0, y0, x1, y1 = polygon.bounds
        for y in np.arange(y0 + 0.5 * spacing, y1, spacing):
            line = LineString([(x0, y), (x1, y)])
            intersection = polygon.intersection(line)
            out = out.add(Drawing.from_shapely(intersection))
        return out.rotate(angle, anchor=centroid.coords[:][0])
