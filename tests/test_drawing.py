from axi import Drawing


def test_repeat():
    d = Drawing([[(1, 2), (3, 4)], [(5, 6), (7, 8), (9, 10)]])
    r = d.repeat()
    assert r.paths == [
        [(1, 2), (3, 4), (1, 2)],
        [(5, 6), (7, 8), (9, 10), (7, 8), (5, 6)],
    ]


def test_repeat_thrice():
    d = Drawing([[(1, 2), (3, 4)], [(5, 6), (7, 8), (9, 10)]])
    r = d.repeat(3)
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
