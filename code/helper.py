# from common import config, VehicleState
import numpy as np
from sympy import symbols, solve, diff

class Helper(object):
    """docstring for Helper"""
    @staticmethod
    def getTm(ove, ove_prev):
        its = config.merging_len if ove.vehicle.lane != ove_prev.vehicle.lane \
            else config.delta
        return max(ove_prev.tm + its / config.case_speed['speed_merge'], ove.t0 + ove.min_pass_time)

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
    def getTmOptimal2(v0, vt, p0, pt, delay=0.0):
        t = symbols('t')
        a = (12.0 * (t * (p0 - pt + t*v0) - 0.5*(t**2)*(v0 - vt)))/(t**4)
        b = -(4 * (1.5*(p0 - pt) + t*v0 + 0.5*t*vt))/(t**2)
        j = ((a*t + b)**3 - b**3) / a
        djdt = diff(j, t)
        res = solve(djdt, t)
        return float(min(res)) + delay

    @staticmethod
    def getTmOptimal(v0, vt, p0, pt, delay=0.0):
        ai, bi, tm = symbols('ai bi tm')
        results = solve((1/2*ai*tm**2 + bi*tm + v0 - vt,
            1/6*ai*tm**3 + 1/2*bi*tm**2 + v0*tm + p0 - pt,
            -1.5*(ai*tm + bi)**2 + 2*ai*vt),
            (ai,bi,tm), dict=True)
        # return result with minimum tm
        try:
            res = min([r[tm] for r in results])
        except Exception as e:
            print("v0={},vt={},p0={},pt={}".format(v0,vt,p0,pt))
            raise e
        return float(res) + delay

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
            0, config.case_speed['speed_merge']])

    @staticmethod
    def updateAVP(v, t):
        if v.state == VehicleState.ON_RAMP:
            ai, bi, ci, di = v.ParaV
            v.vehicle.acceleration = ai * t + bi
            v.vehicle.speed += (v.vehicle.acceleration * config.time_meta)
            v.vehicle.position += (v.vehicle.speed * config.time_meta)
        elif v.state == VehicleState.ON_MERGING:
            v.vehicle.acceleration = 0
            v.vehicle.speed = v.speed_history[-1]
            v.vehicle.position += (v.vehicle.speed * config.time_meta)

        # Fuel consumption
        s = v.vehicle.speed
        a = v.vehicle.acceleration
        fc = 0.1569 * 2.45e-2 * s - 7.415e-4 * s**2 + 5.975e-5 * s**3
        fa = a * (0.07224 + 9.681e-2 * a + 1.075e-3 * a**2)
        fuel = (fc + fa) * config.time_meta

        print("Time: {:.2f}, vehicle: {}, position: {:.3f}, speed: {:.3f}, acc: {:.3f}".format(t,
            v.vehicle.ID, v.vehicle.position, v.vehicle.speed, v.vehicle.acceleration))
        v.acc_history.append(v.vehicle.acceleration)
        v.speed_history.append(v.vehicle.speed)
        v.position_history.append(v.vehicle.position)
        v.fuel_history.append(v.fuel_history[-1] + fuel)
        v.time_steps.append(t)
