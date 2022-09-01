from axi.paths import reloop_paths


def test_reloop():
    paths = [[(0, 0), (2, 0), (1, 1), (0, 0)], [(5, 5), (6, 6), (3, 10)]]
    for _ in range(100):
        new_paths = reloop_paths(paths)
        assert paths[1] in new_paths
        for coord in paths[0]:
            assert coord in new_paths[0]
        assert new_paths[0][0] == new_paths[0][-1]