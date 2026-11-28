from rainbow_web.system import cpu_percent, cpu_times, meminfo, throttled, tree_ticks, wifi

STAT_1 = """cpu  100 0 100 800 0 0 0 0 0 0
cpu0 25 0 25 200 0 0 0 0 0 0
cpu1 25 0 25 200 0 0 0 0 0 0
intr 1 2 3
"""
STAT_2 = """cpu  200 0 200 1400 0 0 0 0 0 0
cpu0 75 0 75 250 0 0 0 0 0 0
cpu1 25 0 25 400 0 0 0 0 0 0
"""


def test_cpu_percent():
    a, b = cpu_times(STAT_1), cpu_times(STAT_2)
    assert len(a) == 3
    assert cpu_percent(a, b) == [25, 67, 0]
    assert cpu_percent(a, a) == [None, None, None]


def test_meminfo():
    text = 'MemTotal:  8257536 kB\nMemFree: 100 kB\nMemAvailable:  5449728 kB\n'
    assert meminfo(text) == (2742, 8064)


def test_throttled():
    assert throttled(0) == ([], [])
    now, past = throttled(0xf0000)
    assert now == [] and past == ['under-voltage', 'clock capped', 'throttled',
                                  'soft temp limit']
    assert throttled(0x50005) == (['under-voltage', 'throttled'],
                                  ['under-voltage', 'throttled'])


def test_wifi():
    text = """Inter-| sta-|   Quality        |   Discarded packets
 face | tus | link level noise |  nwid  crypt   frag  retry   misc | beacon | 22
 wlan0: 0000   48.  -62.  -256        0      0      0  95047      0        0
"""
    assert wifi(text) == (48, -62)
    assert wifi(text, 'wlan1') is None


def test_tree_ticks():
    table = {1: (0, 5), 10: (1, 7), 11: (10, 3), 20: (1, 100)}
    assert tree_ticks(table, 10) == 10
    assert tree_ticks(table, 1) == 115
    assert tree_ticks(table, 99) == 0
