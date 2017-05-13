from collections import deque
import numpy as np
from abc import ABCMeta, abstractmethod
import random
random.seed(42)

# from common import config, VehicleState
# from helper import Helper


class Vehicle(object):
    """docstring for Vehicle"""
    def __init__(self, builder):
        super(Vehicle, self).__init__()
        self.ID = builder.ID
        self.size = builder.size
        self.speed = builder.speed
        self.acceleration = builder.acceleration
        self.max_speed = builder.max_speed
        self.min_speed = builder.min_speed
        self.max_acc = builder.max_acc
        self.lane = builder.lane
        self.position = builder.position


class VehicleBuilder(object):
    """docstring for VehicleBuilder"""
    def __init__(self, ID):
        super(VehicleBuilder, self).__init__()
        self.ID = ID
        self.max_acc = config.max_acc
        self.max_speed = config.max_speed
        self.min_speed = config.min_speed
        self.size = config.size

    def setSpeed(self, speed):
        self.speed = speed
        return self

    def setPosition(self, position):
        self.position = position
        return self

    def setLane(self, lane):
        self.lane = lane
        return self

    def setAcceleration(self, acceleration):
        self.acceleration = acceleration
        return self

    def build(self):
        return Vehicle(self)


class OnBoardVehicle(object):
    """docstring for OnBoardVehicle"""
    def __init__(self, vehicle, t0, min_pass_time=config.min_pass_time):
        self.vehicle = vehicle
        self.t0 = t0
        self.tm = float('Inf')
        self.position_history = [vehicle.position]
        self.speed_history = [vehicle.speed]
        self.acc_history = [vehicle.acceleration]
        self.fuel_history = [0]
        self.time_steps = [t0]
        self.ParaV = None
        self.min_pass_time = min_pass_time
        self.state = VehicleState.PENDING

    @property
    def merge_time(self):
        return self.time_steps[-1] - self.time_steps[0]

    @property
    def tf(self):
        return self.time_steps[-1]

    @property
    def average_speed(self):
        return np.array(self.speed_history).mean()

    @property
    def fuel_consumption(self):
        return self.fuel_history[-1]


class VehicleGeneratorBase(metaclass=ABCMeta):
    """docstring for VehicleGeneratorBase"""
    def __init__(self):
        self.schedule = deque()
        self.buildSchedule()

    @abstractmethod
    def buildSchedule(self):
        pass

    def getAtTime(self, ctime):
        vehicle_to_add = []
        while self.hasVehicle() and self.schedule[0].t0 <= ctime:
            vehicle_to_add.append(self.schedule.popleft())
        return vehicle_to_add

    def hasVehicle(self):
        return len(self.schedule) > 0

    def FIFOIDAssigner(self):
        self.schedule = deque(sorted(self.schedule, key=lambda x:x.t0))
        for i, ove in enumerate(self.schedule):
            ove.vehicle.ID = i

    def SpeedIDAssigner(self):
        # self.schedule = deque(sorted(self.schedule, key=lambda x:x.t0 +
        #     Helper.getTmOptimal2(x.vehicle.speed, config.case_speed['speed_merge'],
        #         -config.control_len, 0)))
        self.schedule = deque(sorted(self.schedule, key=lambda x:x.t0 + x.min_pass_time))
        for i, ove in enumerate(self.schedule):
            ove.vehicle.ID = i
        self.schedule = deque(sorted(self.schedule, key=lambda x:x.t0))


class Case1VehicleGenerator(VehicleGeneratorBase):
    """docstring for Case1VehicleGenerator"""
    def __init__(self):
        super(Case1VehicleGenerator, self).__init__()

    def buildSchedule(self):
        for i in range(config.case1['total_cars']):
            v = VehicleBuilder(i)\
                .setSpeed(config.case1['speed'])\
                .setPosition(-config.control_len)\
                .setAcceleration(0)\
                .setLane(i % 2).build()
            t = 10.0 if (i < 2) else 12.0
            self.schedule.append(OnBoardVehicle(v, t, config.control_len / config.case1['speed']))
        # self.schedule.sort(key=lambda x: x.vehicle.ID)


class Case2VehicleGenerator(VehicleGeneratorBase):
    """docstring for Case1VehicleGenerator"""
    def __init__(self):
        super(Case2VehicleGenerator, self).__init__()

    def buildSchedule(self):
        tnum = config.case2['total_cars']
        # Randomly generate tnum//2 cars on each lane, with time span 10~50
        # This does not ensure feasibility at initial state
        t0_lane0 = np.random.rand(tnum//2) * 40.0 + 10.0
        t0_lane1 = np.random.rand(tnum//2) * 40.0 + 10.0
        for i in range(tnum):
            v = VehicleBuilder(-1)\
                .setSpeed(config.case2['speed'])\
                .setPosition(-config.control_len)\
                .setAcceleration(0)\
                .setLane(i % 2).build()
            t = t0_lane0[i//2] if (i % 2 == 0) else t0_lane1[i//2]
            self.schedule.append(OnBoardVehicle(v, t, config.control_len / config.case2['speed']))
        self.FIFOIDAssigner()


class PoissonVehicleGenerator(VehicleGeneratorBase):
    """docstring for Case1VehicleGenerator"""
    def __init__(self, tnum_lane0, tnum_lane1):
        self.tnum_lane0 = tnum_lane0
        self.tnum_lane1 = tnum_lane1
        super(PoissonVehicleGenerator, self).__init__()

    def buildSchedule(self):
        speed_lane0 = config.case_speed['speed_init_lane0']
        speed_lane1 = config.case_speed['speed_init_lane1']
        t0_lane0 = [0]
        t0_lane1 = [0]
        tsp_lane0 = (config.delta) / speed_lane0
        tsp_lane1 = (config.delta) / speed_lane1
        for i in range(self.tnum_lane0):
            t0_lane0.append(t0_lane0[-1] + tsp_lane0 + np.random.exponential(5.0))
        for i in range(self.tnum_lane1):
            t0_lane1.append(t0_lane1[-1] + tsp_lane1 + np.random.exponential(5.0))
        t0_lane0 = np.array(t0_lane0[1:])
        t0_lane1 = np.array(t0_lane1[1:])

        for i in range(self.tnum_lane0):
            speed_v = speed_lane0 + np.random.randn()
            v = VehicleBuilder(-1)\
                .setSpeed(speed_v)\
                .setPosition(-config.control_len)\
                .setAcceleration(0)\
                .setLane(0).build()
            t = t0_lane0[i]
            self.schedule.append(OnBoardVehicle(v, t,
                Helper.getTmOptimal2(speed_v, config.case_speed['speed_merge'],
                    -config.control_len, 0)))

        for i in range(self.tnum_lane1):
            speed_v = speed_lane1 + np.random.randn()
            v = VehicleBuilder(-1)\
                .setSpeed(speed_v)\
                .setPosition(-config.control_len)\
                .setAcceleration(0)\
                .setLane(1).build()
            t = t0_lane1[i]
            self.schedule.append(OnBoardVehicle(v, t,
                Helper.getTmOptimal2(speed_v, config.case_speed['speed_merge'],
                    -config.control_len, 0, 50.0)))

        # self.FIFOIDAssigner()
        self.SpeedIDAssigner()


class APPVehicleGenerator(VehicleGeneratorBase):
    def __init__(self, tnum, id_assigner, min_pass_time):
        self.tnum = tnum
        self.ida = id_assigner
        self.mpt = min_pass_time
        super(APPVehicleGenerator, self).__init__()

    def buildSchedule(self):
        tnum = self.tnum
        t0_lane0 = np.random.rand(tnum//2) * 40.0 + 10.0
        t0_lane1 = np.random.rand(tnum//2) * 40.0 + 10.0
        for i in range(tnum):
            v = VehicleBuilder(-1)\
                .setSpeed(config.case2['speed'])\
                .setPosition(-config.control_len)\
                .setAcceleration(0)\
                .setLane(i % 2).build()
            t = t0_lane0[i//2] if (i % 2 == 0) else t0_lane1[i//2]
            self.schedule.append(OnBoardVehicle(v, t, self.mpt))
        if self.ida == 'FIFO':
            self.FIFOIDAssigner()
        elif self.ida == 'main':
            self.MainRoadFirstIDAssigner()


class GameLoop(object):
    """docstring for GameLoop"""
    def __init__(self, vscd):
        super(GameLoop, self).__init__()
        self.ctime = 0
        self.vscd = vscd
        self.on_board_vehicles = deque()
        self.finished_vehicels = deque()

    def isOver(self):
        if self.ctime >= config.max_sim_time:
            print("Simulation time out.")
            return True
        return (not self.vscd.hasVehicle()) and self.isEmpty()

    def isEmpty(self):
        return len(self.on_board_vehicles) == 0

    def nextStep(self):
        self.ctime += config.time_meta
        t = self.ctime
        ove_t = self.vscd.getAtTime(t)
        for v in ove_t:
            if v.vehicle.ID == 0:
                v.tm = v.t0 + max(config.min_pass_time, v.min_pass_time)
            elif len(self.on_board_vehicles) > 0:
                v.tm = Helper.getTm(v, self.on_board_vehicles[-1])
            else:
                v.tm = Helper.getTm(v, self.finished_vehicels[-1])
            self.on_board_vehicles.append(v)
            TimeM = Helper.getTimeMatrix(t, v.tm)
            ConfV = Helper.getConfigVec(v)
            v.ParaV = np.dot(np.linalg.inv(TimeM), ConfV)
            v.state = VehicleState.ON_RAMP

        for v in self.on_board_vehicles:
            Helper.updateAVP(v, t)
            if v.vehicle.position >= 0:
                v.state = VehicleState.ON_MERGING

        while not self.isEmpty() and self.on_board_vehicles[0].vehicle.position >= config.merging_len:
            self.on_board_vehicles[0].state = VehicleState.FINISHED
            self.finished_vehicels.append((self.on_board_vehicles.popleft()))

    def play(self):
        while not self.isOver():
            self.nextStep()
        self.measure()

    def measure(self):
        # Measure average merging time
        AMT = np.array([v.merge_time for v in self.finished_vehicels]).mean()
        # Measure traffic flow
        TF = length(self.finished_vehicels) / (self.finished_vehicels[-1].tf - self.finished_vehicels[0].t0)
        # Measure average speed
        AS = 1 / np.array([1 / v.average_speed for v in self.finished_vehicels]).mean() * 3.6
        # Measure average fuel consumption
        AFC = np.array([v.fuel_consumption for v in self.finished_vehicels]).mean()
        print("""Average merging time: {} s\n
            Traffic flow: {} vehicle/s\n
            Average speed: {} km/h\n
            Average fuel consumption: {} ml/vehicle""".format(AMT, TF, AS, AFC))

    def draw_result(self, file_path):
        from bokeh.layouts import gridplot
        from bokeh.plotting import figure, output_file, show

        output_file(file_path)
        TOOLS = "pan,wheel_zoom,box_zoom,reset,save,box_select,lasso_select"
        s1 = figure(tools=TOOLS, title="positions", x_axis_label="time", y_axis_label="position")
        for tv in self.finished_vehicels:
            s1.line(tv.time_steps, tv.position_history,
                line_color="red" if tv.vehicle.lane == 0 else "blue",
                legend="Car {}".format(tv.vehicle.ID), line_width=2)
        s1.xgrid.minor_grid_line_color = 'navy'
        s1.xgrid.minor_grid_line_alpha = 0.1
        s1.ygrid.minor_grid_line_color = 'navy'
        s1.ygrid.minor_grid_line_alpha = 0.1

        s2 = figure(tools=TOOLS, title="speed", x_axis_label="time", y_axis_label="speed",
            x_range=s1.x_range)
        for tv in self.finished_vehicels:
            s2.line(tv.time_steps, tv.speed_history,
                line_color="red" if tv.vehicle.lane == 0 else "blue",
                legend="Car {}".format(tv.vehicle.ID), line_width=2)
        s2.xgrid.minor_grid_line_color = 'navy'
        s2.xgrid.minor_grid_line_alpha = 0.1
        s2.ygrid.minor_grid_line_color = 'navy'
        s2.ygrid.minor_grid_line_alpha = 0.1

        s3 = figure(tools=TOOLS, title="acceleration", x_axis_label="time", y_axis_label="acceleration",
            x_range=s1.x_range)
        for tv in self.finished_vehicels:
            s3.line(tv.time_steps, tv.acc_history,
                line_color="red" if tv.vehicle.lane == 0 else "blue",
                legend="Car {}".format(tv.vehicle.ID), line_width=2)
        s3.xgrid.minor_grid_line_color = 'navy'
        s3.xgrid.minor_grid_line_alpha = 0.1
        s3.ygrid.minor_grid_line_color = 'navy'
        s3.ygrid.minor_grid_line_alpha = 0.1

        s4 = figure(tools=TOOLS, title="fuel consumption", x_axis_label="time", y_axis_label="fuel",
            x_range=s1.x_range)
        for tv in self.finished_vehicels:
            s4.line(tv.time_steps, tv.fuel_history,
                line_color="red" if tv.vehicle.lane == 0 else "blue",
                legend="Car {}".format(tv.vehicle.ID), line_width=2)
        s4.xgrid.minor_grid_line_color = 'navy'
        s4.xgrid.minor_grid_line_alpha = 0.1
        s4.ygrid.minor_grid_line_color = 'navy'
        s4.ygrid.minor_grid_line_alpha = 0.1

        p = gridplot([[s1, s2, s3, s4]])
        try:
            show(p)
        except Exception as e:
            pass
        show(p)


class SpeedGameLoop(GameLoop):
    """docstring for SpeedGameLoop"""
    def __init__(self, vscd):
        super(SpeedGameLoop, self).__init__(vscd)
        self.on_board_vehicles = []

    def nextStep(self):
        self.ctime += config.time_meta
        t = self.ctime
        ove_t = self.vscd.getAtTime(t)
        for v in ove_t:
            tmp_v_stack = []
            while len(self.on_board_vehicles) > 0 and self.on_board_vehicles[-1].vehicle.ID > v.vehicle.ID:
                tmpv = self.on_board_vehicles.pop()
                tmpv.t0 = t
                tmpv.min_pass_time = max(tmpv.min_pass_time, Helper.getTmOptimal2(tmpv.vehicle.speed,
                    config.case_speed['speed_merge'], tmpv.vehicle.position, 0))
                tmp_v_stack.append(tmpv)

            # Get t_m
            if len(self.on_board_vehicles) == 0 and len(self.finished_vehicels) == 0:
                v.tm = v.t0 + max(config.min_pass_time, v.min_pass_time)
            elif len(self.on_board_vehicles) > 0:
                v.tm = Helper.getTm(v, self.on_board_vehicles[-1])
            else:
                v.tm = Helper.getTm(v, self.finished_vehicels[-1])
            tmp_v_stack.append(v)
            prevve = None
            for i in reversed(range(len(tmp_v_stack))):
                ve = tmp_v_stack[i]
                if prevve is not None:
                    ve.tm = Helper.getTm(ve, prevve)
                self.on_board_vehicles.append(ve)
                TimeM = Helper.getTimeMatrix(t, ve.tm)
                ConfV = Helper.getConfigVec(ve)
                # from IPython import embed; embed()
                ve.ParaV = np.dot(np.linalg.inv(TimeM), ConfV)
                ve.state = VehicleState.ON_RAMP
                prevve = ve

        for v in self.on_board_vehicles:
            Helper.updateAVP(v, t)
            if v.vehicle.position >= 0:
                v.state = VehicleState.ON_MERGING

        while not self.isEmpty() and self.on_board_vehicles[0].vehicle.position >= config.merging_len:
            self.on_board_vehicles[0].state = VehicleState.FINISHED
            self.finished_vehicels.append((self.on_board_vehicles.pop(0)))


def main():
    # vehicle_generator = Case2VehicleGenerator()
    # game = GameLoop(vehicle_generator)
    # game.play()
    # game.draw_result("result.html")

    # vehicle_generator = APPVehicleGenerator(12, 'FIFO', 16.9)
    vehicle_generator = PoissonVehicleGenerator(config.case_speed['tnum_lane0'],
        config.case_speed['tnum_lane1'])
    ggame = SpeedGameLoop(vehicle_generator)
    ggame.play()
    ggame.draw_result("result.html")


if __name__ == '__main__':
    main()