from typing import Union
from xml.dom import minidom

import numpy as np
from axi import Drawing

from axi.paths import Point, Path
from shapely.geometry import MultiLineString, LineString
from svg.path import parse_path, Line, CubicBezier, Close
from bezier.curve import Curve

from axi.renderer import render_gl


def complex_tuple(n: complex) -> Point:
    return n.real, n.imag


def svg_line_parse(line: Union[Line, Close]) -> Path:
    return [complex_tuple(line.start), complex_tuple(line.end)]


def svg_cubic_bezier_parse(bezier: CubicBezier, n: int = 128) -> Path:
    nodes = [complex_tuple(p) for p in (bezier.start, bezier.control1, bezier.control2, bezier.end)]
    nodes = np.array(nodes).T
    curve = Curve.from_nodes(nodes)
    points = curve.evaluate_multi(np.linspace(0, 1, n, endpoint=True))
    return [tuple(row) for row in points.T]


def load_svg(path: str) -> MultiLineString:
    with open(path) as f:
        doc = minidom.parse(f)
    paths = [parse_path(path.getAttribute('d')) for path in doc.getElementsByTagName('path')]
    doc.unlink()
    line_strings: list[LineString] = []
    for path in paths:
        path_points: Path = []
        for elem in path:
            print(elem)
            if isinstance(elem, Line) or isinstance(elem, Close):
                path_points.extend(svg_line_parse(elem))
            elif isinstance(elem, CubicBezier):
                path_points.extend(svg_cubic_bezier_parse(elem))
        path_points = [path_points[i] for i in range(len(path_points) - 1) if path_points[i] != path_points[i + 1]] + \
                      [path_points[-1]]
        line_strings.append(LineString(path_points))
    return MultiLineString(line_strings)


def main():
    shape = load_svg("C:\\Users\\eliot\\Desktop\\drawing.svg")
    d = Drawing.from_shapely(shape).scale_to_fit(8, 8, 0.5).center(8, 8)
    render_gl([d], 8, 8, dpi=100)


if __name__ == '__main__':
    main()
