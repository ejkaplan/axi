from __future__ import division

from itertools import chain

from matplotlib import pyplot as plt, patches
from matplotlib.path import Path as PltPath
from pyglet import window, gl, app
from pyglet.graphics import Batch, Group

from axi import Drawing

colors = [
    (0, 0, 255, 255),  # blue
    (255, 0, 0, 255),  # red
    (0, 82, 33, 255),  # dark green
    (255, 123, 0, 255),  # orange
    (8, 142, 149, 255),  # aqua
    (255, 0, 255, 255),  # fuchsia
    (158, 253, 56, 255),  # lime
    (206, 81, 113, 255),  # hot pink
]


def batch_drawings(drawings: list[Drawing], width: float, height: float,
                   dpi: float) -> Batch:
    assert len(drawings) <= len(colors)
    batch = Batch()
    for i, drawing in enumerate(drawings):
        color = colors[i]
        for path in drawing.paths:
            grp = Group()
            path = [(dpi * x, dpi * (height - y)) for x, y in path]
            vertices = path[0] + tuple(chain(*path)) + path[-1]
            batch.add(len(vertices) // 2, gl.GL_LINE_STRIP, grp, ('v2f', vertices),
                      ('c4B', color * (len(vertices) // 2)))
    return batch


def render_gl(drawings: list[Drawing], width: float, height: float, dpi=128):
    batch = batch_drawings(drawings, width, height, dpi)
    config = gl.Config(sample_buffers=1, samples=8, double_buffer=True)
    win = window.Window(int(width * dpi), int(height * dpi), "plot preview",
                        config=config)

    @win.event
    def on_draw():
        gl.glEnable(gl.GL_LINE_SMOOTH)
        win.clear()
        batch.draw()

    app.run()


@staticmethod
def render_matplotlib(layers: list[Drawing], width: float, height: float):
    colors = [
        "blue",
        "red",
        "darkgreen",
        "orange",
        "aqua",
        "fuchsia",
        "lime",
        "hotpink",
    ]
    fig, ax = plt.subplots()
    border = PltPath(
        [(0, 0), (width, 0), (width, height), (0, height), (0, 0)],
        [
            PltPath.MOVETO,
            PltPath.LINETO,
            PltPath.LINETO,
            PltPath.LINETO,
            PltPath.CLOSEPOLY,
        ],
    )
    ax.add_patch(patches.PathPatch(border, edgecolor="black", facecolor="white"))
    for i, layer in enumerate(layers):
        codes = []
        coords = []
        for path in layer.paths:
            codes += [PltPath.MOVETO] + ([PltPath.LINETO] * (len(path) - 1))
            coords += [(x, height - y) for x, y in path]
        plt_path = PltPath(coords, codes)
        ax.add_patch(
            patches.PathPatch(plt_path, edgecolor=colors[i], fill=False)
        )

    plt.xlim([-0.5, width + 0.5])
    plt.ylim([-0.5, height + 0.5])
    ax.axis("equal")
    plt.show()
