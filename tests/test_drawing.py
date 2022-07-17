from pytest import fixture

from axi import Drawing


@fixture
def drawing() -> Drawing:
    return Drawing([[(1, 2), (3, 4)], [(5, 6), (7, 8), (9, 10)]])


def test_repeat(drawing):
    r = drawing.repeat()
    assert r.paths == [
        [(1, 2), (3, 4), (1, 2)],
        [(5, 6), (7, 8), (9, 10), (7, 8), (5, 6)],
    ]


def test_repeat_thrice(drawing):
    r = drawing.repeat(3)
    assert r.paths == [
        [(1, 2), (3, 4), (1, 2), (3, 4)],
        [
            (5, 6),
            (7, 8),
            (9, 10),
            (7, 8),
            (5, 6),
            (7, 8),
            (9, 10),
        ],
    ]


def test_null_repeat(drawing):
    r = drawing.repeat(1)
    assert r.paths == [[(1, 2), (3, 4)], [(5, 6), (7, 8), (9, 10)]]
