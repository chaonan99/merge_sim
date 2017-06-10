from common import config, VehicleState
import numpy as np
from sympy import symbols, solve, solveset, S, diff
from sympy.solvers.solveset import solveset_real
import math

class Helper(object):
    """docstring for Helper"""
    @staticmethod
    def getTm(ove, ove_prev, mode="hard"):
        its = config.merging_len if ove.vehicle.lane != ove_prev.vehicle.lane \
            else config.delta
        if mode == "soft":
            timdelay = ove.t0 + ove.min_pass_time if ove.vehicle.lane == 1 else 0
            return max(ove_prev.tm + its / config.case_speed['speed_merge'], timdelay)
        elif mode == "hard":
            return ove_prev.tm + its / config.case_speed['speed_merge']
        else:
            assert False, "Invalid mode: {}".format(mode)
        # return max(ove_prev.tm + its / config.case_speed['speed_merge'], ove.t0 + Helper.getTc(ove.vehicle))

    @staticmethod
    def getTc(ve):
        # Calculate the minimum arrival time for a single vehicle
        # Can be used as the `min_pass_time` for an `onBoardVehicle`
        L = config.control_len
        vm = config.max_speed
        v0 = ve.speed
        um = config.max_acc
        vd = config.speed_merge
        # L = 107.5   # This example yields `tim1 = tim2 = 5.0`
        # vm = 30.0
        # v0 = 10.0
        # vd = 25.0
        # um = 5.0
        tim1 = (L/vm) + (vm-vd-v0)/um + (v0**2+vd**2)/(2*um*vm)
        tim2 = (math.sqrt(2*(2*um*L + v0**2 + vd**2)) - v0 - vd)/um
        return max(tim1, tim2)

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
        try:
            res = solveset_real(djdt, t)
        except Exception as e:
            res = solve(djdt, t)
        print("Calculating tm for v0={}, vt={}, p0={}, pt={}, tm={}".format(v0, vt, p0, pt, float(min(res))))
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
    def solveTmNoConstraint(ove):
        tm = ove.tm - ove.tf   # tm left before merging
        v0 = ove.v0
        vd = config.speed_merge
        L = config.control_len
        tm = 25.0   # tm left before merging
        v0 = 10.0
        vd = 25.0
        L = 720
        a, b = symbols('a b')
        result = solve((
            1/6*a*tm**3 + 1/2*b*tm**2 + v0*tm - L,
            1/2*a*tm**2 + b*tm + v0 - vd),
        (a, b),dict=True)[0]
        return {'a':float(result[a]), 'b':float(result[b])}

    def solveTmVMaxMin(ove, vmax=True):
        tm = ove.tm - ove.tf   # tm left before merging
        v0 = ove.v0
        vd = config.speed_merge
        vm = config.max_speed if vmax else config.min_speed
        L = config.control_len
        # tm = 25.0   # result correct: a=-2.5, b=10.0, tc=6.0
        # v0 = 10.0
        # vd = 25.0
        # L = 720
        # vm = 30.0
        a, b, tc = symbols('a b tc')
        results = solve((
            1/2*a*tc**2 + b*tc + v0 - vd,
            v0 - b**2/(2*a) - vm,
            1/6*a*tc**3 + 1/2*b*tc**2 + v0*tc - L + (tm - tc)*vm),
        (a, b, tc), dict=True)
        for res in results:
            an = float(res[a])
            bn = float(res[b])
            tcn = float(res[tc])
            if tcn >= 0 and tcn <= tm and bn*(an*tcn + bn) < 0:
                return {'a':an, 'b':bn, 'tc':tcn}
        assert False, "Max/min v constraint cannot solve!"

    # def solveTmABegin(ove):

    """Sympy cannot solve this equation..."""
    # def solveTmAVBegin(ove, amax=True):
    #     tm = ove.tm - ove.tf
    #     v0 = ove.v0
    #     vd = config.speed_merge
    #     vm = config.max_speed if amax else config.min_speed
    #     L = config.control_len
    #     um = config.max_acc if amax else config.min_acc
    #     tm = 25.0
    #     v0 = 10.0
    #     vd = 25.0
    #     L = 720
    #     vm = 30.0
    #     um = 8.0
    #     a, c, ta, tc = symbols('a c ta tc')
    #     results = solveset([
    #         1/2*a*(tc**2) + um*tc + c - vd,
    #         -(um**2)/(2*a) + c - vm,
    #         1/6*a*(tc**3) + 1/2*um*(tc**2) + c*tc + 1/2*um*(ta**2) + v0*ta + vm*(tm - ta - tc) - L,
    #         um*ta + v0 - c],
    #     (a, c, ta, tc), domain=S.Reals)

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

        # print("Time: {:.2f}, vehicle: {}, position: {:.3f}, speed: {:.3f}, acc: {:.3f}".format(t,
        #     v.vehicle.ID, v.vehicle.position, v.vehicle.speed, v.vehicle.acceleration))
        v.acc_history.append(v.vehicle.acceleration)
        v.speed_history.append(v.vehicle.speed)
        v.position_history.append(v.vehicle.position)
        v.fuel_history.append(v.fuel_history[-1] + fuel)
        v.time_steps.append(t)
