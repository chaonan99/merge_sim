from enum import Enum

class Config(object):
    control_len = 400.0
    merging_len = 30.0
    max_acc = 2.5
    size = (3.0, 1.8)
    delta = 20.0  # safe distance between vehicles
    time_meta = 0.2
    max_sim_time = 100.0

    case1 = {
        'total_cars': 4,
        'speed': 13.4,
        'tm_0': 30.0,
    }
    case2 = {
        'total_cars': 30,
        'speed': 13.4,
        'tm_0': 30.0
    }


class VehicleState(Enum):
    PENDING = 1
    ON_RAMP = 2
    ON_MERGING = 3
    FINISHED = 4


config = Config()