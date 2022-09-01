from typing import Optional

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
