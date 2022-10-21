from typing import Optional

from shapely.affinity import scale
from shapely.geometry import LineString, Polygon, MultiLineString, MultiPolygon, \
    GeometryCollection

from . import Drawing
from .device import Device


def reset():
    d = Device()
    d.disable_motors()
    d.pen_up()


def draw(
        drawing: Drawing, progress: bool = True, device: Optional[Device] = None
) -> Device:
    device = Device() if device is None else device
    device.enable_motors()
    device.run_drawing(drawing, progress)
    device.disable_motors()
    return device


def draw_layers(
        layers: list[Drawing], progress: bool = True, device: Optional[Device] = None
):
    device = Device() if device is None else Device
    for i, layer in enumerate(layers):
        input(f"Press enter when you're ready to draw layer {i}.")
        draw(layer, progress, device=device)


Geometry = LineString | Polygon | MultiLineString | MultiPolygon | GeometryCollection


def shapely_scale_to_fit(geometry: Geometry, width: float, height: float) -> Geometry:
    minx, miny, maxx, maxy = geometry.bounds
    gw, gh = maxx - minx, maxy - miny
    if gw == gh == 0:
        return geometry
    if gw == 0:
        scale_factor = height / gh
    elif gh == 0:
        scale_factor = width / gw
    else:
        scale_factor = min(width / gw, height / gh)
    return scale(geometry, scale_factor, scale_factor)


