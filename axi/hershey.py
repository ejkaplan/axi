from __future__ import division

import itertools
from string import printable

from .drawing import Drawing, Path
from .hershey_fonts import *

HersheyFont = list[tuple[float, float, list[list[tuple[float, float]]]]]


def text(string: str, font: HersheyFont = FUTURAL, spacing: float = 0, extra: float = 0) -> list[Path]:
    result = []
    x = 0
    for ch in string:
        index = ord(ch) - 32
        if index < 0 or index >= 96:
            x += spacing
            continue
        lt, rt, coords = font[index]
        for path in coords:
            path = [(x + i - lt, j) for i, j in path]
            if path:
                result.append(path)
        x += rt - lt + spacing
        if index == 0:
            x += extra
    return result


def _word_wrap(string: str, width: float, measure_func) -> list[str]:
    result = []
    for line in string.split('\n'):
        fields = itertools.groupby(line, lambda x: x.isspace())
        fields = [''.join(g) for _, g in fields]
        if len(fields) % 2 == 1:
            fields.append('')
        x = ''
        for a, b in zip(fields[::2], fields[1::2]):
            w, _ = measure_func(x + a)
            if w > width:
                if x == '':
                    result.append(a)
                    continue
                else:
                    result.append(x)
                    x = ''
            x += a + b
        if x != '':
            result.append(x)
    result = [x.strip() for x in result]
    return result


class Font(object):
    def __init__(self, font: HersheyFont, point_size: float):
        self.font = font
        self.max_height = Drawing(text(printable, font)).height
        self.scale = (point_size / 72) / self.max_height

    def text(self, string: str) -> Drawing:
        d = Drawing(text(string, self.font))
        d = d.scale(self.scale)
        return d

    def justify_text(self, string: str, width: float) -> Drawing:
        d = self.text(string)
        w = d.width
        spaces = string.count(' ')
        if spaces == 0 or w >= width:
            return d
        e = ((width - w) / spaces) / self.scale
        d = Drawing(text(string, self.font, extra=e))
        d = d.scale(self.scale)
        return d

    def measure(self, string: str):
        return self.text(string).size

    def wrap(self, string: str, width: float, line_spacing: float = 1,
             align: float = 0, justify: bool = False) -> Drawing:
        lines = _word_wrap(string, width, self.measure)
        ds = [self.text(line) for line in lines]
        max_width = max(d.width for d in ds)
        if justify:
            jds = [self.justify_text(line, max_width) for line in lines]
            ds = jds[:-1] + [ds[-1]]
        spacing = line_spacing * self.max_height * self.scale
        result = Drawing()
        y = 0
        for d in ds:
            if align == 0:
                x = 0
            elif align == 1:
                x = max_width - d.width
            else:
                x = max_width / 2 - d.width / 2
            result.add(d.translate(x, y))
            y += spacing
        return result
