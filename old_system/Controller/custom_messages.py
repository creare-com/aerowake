from collections import OrderedDict

# Custom messages
set_mission_mode = OrderedDict((
    ("TAKE_OFF", 0),
    ("LAND", 1),
    ("START_MISSION", 2),
    ("STOP_MISSION", 3),
    ("LOITER", 4),
    ("EMERGENCY_ALL_STOP", 9),
    ("SCHEDULE_SWEEP", -1)  # Not part of the set_mission_mode message but nice for gui
))

mission_status = OrderedDict((
    (0, "TAKING_OFF"),
    (1, "LANDING"),
    (2, "FLYING MISSION"),
    (3, "STOPPING MISSION"),
    (4, "LOITERING"),
    (9, "EMERGENCY STOP"),
))

def mk_reversed_dict(fd):
    bd = OrderedDict()
    for k, v in fd.iteritems():
        bd[v] = k
    return bd

get_mission_mode = mk_reversed_dict(set_mission_mode)
send_mission_status = mk_reversed_dict(mission_status)
