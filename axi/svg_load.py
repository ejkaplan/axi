from typing import Union
from xml.dom import minidom

import numpy as np
from axi import Drawing

from axi.paths import Point, Path
from shapely.geometry import MultiLineString, LineString, GeometryCollection, Polygon
from svg.path import parse_path, Line, CubicBezier, Close

from axi.renderer import render_gl
from axi.util import shapely_scale_to_fit


def complex_tuple(n: complex) -> Point:
    return n.real, n.imag


def svg_line_parse(line: Union[Line, Close]) -> Path:
    return [complex_tuple(line.start), complex_tuple(line.end)]


def bezier_eval(nodes: np.ndarray, t: np.ndarray) -> np.ndarray:
    return (1 - t) ** 3 * nodes[:, [0]] + 3 * (1 - t) ** 2 * t * nodes[:, [1]] + 3 * (
                1 - t) * t ** 2 * nodes[:, [2]] + t ** 3 * nodes[:, [3]]


def svg_cubic_bezier_parse(bezier: CubicBezier, n: int = 128) -> Path:
    nodes = [complex_tuple(p) for p in
             (bezier.start, bezier.control1, bezier.control2, bezier.end)]
    points = bezier_eval(np.array(nodes).T, np.linspace(0, 1, n))
    return [tuple(p) for p in points.T]


def load_svg(path: str) -> GeometryCollection:
    with open(path) as f:
        doc = minidom.parse(f)
    paths = [parse_path(path.getAttribute('d')) for path in
             doc.getElementsByTagName('path')]
    doc.unlink()
    shapes: list[LineString | Polygon] = []
    for path in paths:
        polygon = False
        path_points: Path = []
        edge: Path = []
        holes: list[Path] = []
        for elem in path:
            if isinstance(elem, Line):
                path_points.extend(svg_line_parse(elem))
            elif isinstance(elem, CubicBezier):
                path_points.extend(svg_cubic_bezier_parse(elem))
            elif isinstance(elem, Close):
                polygon = True
                path_points.extend(svg_line_parse(elem))
                if edge:
                    holes.append(path_points)
                else:
                    edge = path_points
                path_points = []
        if polygon:
            shapes.append(Polygon(edge, holes))
        else:
            shapes.append(LineString(path_points))
    return GeometryCollection(shapes)


def main():
    shapes: GeometryCollection = load_svg("C:\\Users\\eliotkaplan\\Desktop\\drawing.svg")
    shapes = shapely_scale_to_fit(shapes, 7, 7)
    shape = shapes.geoms[0]
    a = Drawing.shade(shape, np.pi/4, 0.1)
    b = Drawing.shade(shape, 3*np.pi/4, 0.1)
    d = a.add(b).center(8, 8)
    render_gl([d], 8, 8, dpi=100)


if __name__ == '__main__':
    main()
