from . import Drawing
from .device import Device


def reset():
    d = Device()
    d.disable_motors()
    d.pen_up()


def draw(drawing: Drawing, progress: bool = True):
    # TODO: support drawing, list of paths, or single path
    d = Device()
    d.enable_motors()
    d.run_drawing(drawing, progress)
    d.disable_motors()


def draw_layers(layers: list[Drawing], progress: bool = True):
    for i, layer in enumerate(layers):
        input(f"Press enter when you're ready to draw layer {i}.")
        draw(layer, progress)
