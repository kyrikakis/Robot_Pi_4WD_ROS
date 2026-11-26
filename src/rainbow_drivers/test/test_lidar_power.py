from rainbow_drivers.lidar_power import lidar_wanted, running_programs

PROGRAMS = ['rainbow_slam', 'rainbow_localization', 'rainbow_navigation']

STATUS = """\
rainbow_localization             STOPPED   Nov 26 07:32 PM
rainbow_navigation               RUNNING   pid 2551, uptime 0:06:06
rainbow_slam                     STARTING
"""


def test_running_programs():
    assert running_programs(STATUS) == ['rainbow_navigation', 'rainbow_slam']
    assert running_programs('') == []


def test_brain_alone_does_not_want_the_lidar():
    assert lidar_wanted(['brain'], [], ['brain'], PROGRAMS) == (False, [])


def test_a_scan_consumer_wants_it():
    wanted, users = lidar_wanted(['brain', 'amcl', 'amcl'], [], ['brain'], PROGRAMS)
    assert wanted and users == ['amcl']


def test_a_running_program_wants_it_without_subscribers():
    wanted, users = lidar_wanted([], ['rainbow_navigation'], ['brain'], PROGRAMS)
    assert wanted and users == ['rainbow_navigation']


def test_other_programs_do_not_count():
    assert lidar_wanted([], ['rainbow_voice'], ['brain'], PROGRAMS) == (False, [])
