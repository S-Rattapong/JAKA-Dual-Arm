from math import radians
from tools.exp2_host_endpoints import endpoints, difference


def motion(start, stop, end=360):
    times = list(range(0, end + 1, 5))
    positions = [[radians(2 * max(0, min(1, (t-start)/(stop-start))))] * 6 for t in times]
    return endpoints(times, positions, [0]*6, [radians(2)]*6, .01, .1, 25, 15)


def test_simultaneous_motion():
    a = motion(50, 200)
    assert all(x is not None for x in a)
    assert difference(a[0], a[0]) == [-5, 5]
    assert difference(a[1], a[1]) == [-5, 5]


def test_delayed_start_common_stop():
    left, right = motion(50, 200), motion(80, 200)
    assert difference(left[0], right[0]) == [25, 35]
    assert difference(left[1], right[1]) == [-5, 5]


def test_delayed_stop_and_missing_tail():
    left, right = motion(50, 200), motion(50, 230)
    assert difference(left[1], right[1]) == [25, 35]
    assert motion(50, 200, end=205)[1] is None
