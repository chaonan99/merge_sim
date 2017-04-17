from common import config, VehicleState
import numpy as np
from sympy import symbols, solve

class Helper(object):
    """docstring for Helper"""
    @staticmethod
    def getTm(ove, ove_prev):
        its = config.merging_len if ove.vehicle.lane != ove_prev.vehicle.lane \
            else config.delta
        return max(ove_prev.tm + its / ove.speed_history[0], ove.t0 + ove.min_pass_time)

    @staticmethod
    def getTmPossSpeed(v0, vm, vt):
        ai, bi, ci, tm = symbols('ai bi ci tm')
        ti = tm / 2
        results = solve((ci - v0,
            1/2*ai*tm**2 + bi*tm + ci - vt,
            1/2*ai*ti**2 + bi*ti + ci - vm,
            1/6*ai*tm**3 + 1/2*bi*tm**2 + ci*tm - config.control_len),
            (ai,bi,ci,tm), dict=True)
        tm_n = 0
        for r in results:
            if abs(r[tm]) > 0:
                tm_n = abs(r[tm])
                break
        assert tm_n > 0, "Could not find valid t_m"
        print("Tm with max speed limit: {}".format(tm_n))
        return float(tm_n)

    @staticmethod
    def getTmPossAcc(v0, ma):
        ai, bi, ci, tm = symbols('ai bi ci tm')
        ti = tm / 2
        results = solve((ci - v0,
            1/2*ai*tm**2 + bi*tm + ci - v0,
            bi - ma,
            1/6*ai*tm**3 + 1/2*bi*tm**2 + ci*tm - config.control_len),
            (ai,bi,ci,tm), dict=True)
        tm_n = 0
        for r in results:
            if abs(r[tm]) > 0:
                tm_n = abs(r[tm])
                break
        assert tm_n > 0, "Could not find valid t_m"
        print("Tm with max acceleration limit: {}".format(tm_n))
        return float(tm_n)

    @staticmethod
    def getTimeMatrix(t, tif):
        return np.array([[(1/6.0)*(t**3), 0.5*(t**2), t, 1],
                         [0.5*(t**2), t, 1, 0],
                         [(1/6.0)*(tif**3), 0.5*(tif**2), tif, 1],
                         [0.5*(tif**2), tif, 1, 0],])

    @staticmethod
    def getConfigVec(ove):
        return np.array([ove.vehicle.position, ove.vehicle.speed,
            0, ove.speed_history[0]])

    @staticmethod
    def updateAVP(v, t):
        if v.state == VehicleState.ON_RAMP:
            ai, bi, ci, di = v.ParaV
            v.vehicle.acceleration = ai * t + bi
            # v.vehicle.acceleration = min(acc, v.vehicle.max_acc) \
            #     if acc >= 0 else max(acc, -v.vehicle.max_acc)
            v.vehicle.speed += (v.vehicle.acceleration * config.time_meta)
            # if speed >= v.vehicle.max_speed:
            #     v.vehicle.speed = v.vehicle.max_speed
            # elif speed <= v.vehicle.min_speed:
            #     v.vehicle.speed = v.vehicle.min_speed
            # else:v.vehicle.speed += (v.vehicle.acceleration * config.time_meta)
            v.vehicle.position += (v.vehicle.speed * config.time_meta)
        elif v.state == VehicleState.ON_MERGING:
            v.vehicle.acceleration = 0
            v.vehicle.speed = v.speed_history[0]
            v.vehicle.position += (v.vehicle.speed * config.time_meta)
            # v.vehicle.speed
        # v.vehicle.speed += (v.vehicle.acceleration * config.time_meta)
        # v.vehicle.position += (v.vehicle.speed * config.time_meta)
        # acc = ai * t + bi
        # v.vehicle.acceleration = min(acc, v.vehicle.max_acc) \
        #     if acc >= 0 else max(acc, -v.vehicle.max_acc)
        # speed = v.vehicle.speed + v.vehicle.acceleration * config.time_meta
        # if speed >= v.vehicle.max_speed:
        #     v.vehicle.speed = v.vehicle.max_speed
        # elif speed <= v.vehicle.min_speed:
        #     v.vehicle.speed = v.vehicle.min_speed
        # else:v.vehicle.speed += (v.vehicle.acceleration * config.time_meta)
        # v.vehicle.position -= (v.vehicle.speed * config.time_meta)
        # # v.vehicle.speed = 0.5 * ai * t**2 + bi * t + ci
        # # v.vehicle.position = (1/6.0) * ai * t**3 + 0.5 * bi * t**2 + ci * t + di
        print("Time: {:.2f}, vehicle: {}, position: {:.3f}, speed: {:.3f}, acc: {:.3f}".format(t,
            v.vehicle.ID, v.vehicle.position, v.vehicle.speed, v.vehicle.acceleration))
        v.acc_history.append(v.vehicle.acceleration)
        v.speed_history.append(v.vehicle.speed)
        v.position_history.append(v.vehicle.position)
        v.time_steps.append(t)
