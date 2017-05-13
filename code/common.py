from enum import Enum

class Config(object):
    control_len = 400.0
    merging_len = 30.0
    max_acc = 3.0         # maximum acceleration
    max_speed = 30.0      # maximum speed
    min_speed = 5.0
    size = (3.0, 1.8)     # size of the vehicle
    delta = 10.0          # safe distance between vehicles
    time_meta = 0.1      # time meta for simulation
    max_sim_time = 400.0
    min_pass_time = 0.0

    case1 = {
        'total_cars': 4,
        'speed': 13.4,
    }
    case2 = {
        'total_cars': 30,
        'speed': 13.4,
    }
    case_speed = {
        'tnum_lane0': 15,
        'tnum_lane1': 10,
        'speed_init_lane0': 25.0,
        'speed_init_lane1': 12.5,
        'speed_merge': 25.0
    }


class VehicleState(Enum):
    PENDING = 1
    ON_RAMP = 2
    ON_MERGING = 3
    FINISHED = 4


config = Config()