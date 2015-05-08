# Custom messages
set_mission_mode = {
    "TAKE_OFF": 0,
    "LAND": 1,
    "START_MISSION": 2,
    "STOP_MISSION": 3,
    "LOITER": 4,
    "EMERGENCY_ALL_STOP": 9,
    "SCHEDULE_SWEEP": -1  # Not part of the set_mission_mode message but nice for gui
}

mission_status = {
    0: "TAKING_OFF",
    1: "LANDING",
    2: "FLYING MISSION",
    3: "STOPPING MISSION",
    4: "LOITERING",
    9: "EMERGENCY STOP",
}

