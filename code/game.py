from collections import deque
import numpy as np
from abc import ABCMeta, abstractmethod

from common import config, VehicleState
from helper import Helper


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
        self.max_speed = float('Inf')
        self.min_speed = 0
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
    def __init__(self, vehicle, t0):
        self.vehicle = vehicle
        self.t0 = t0
        self.tm = float('Inf')
        self.position_history = [vehicle.position]
        self.speed_history = [vehicle.speed]
        self.acc_history = [vehicle.acceleration]
        self.time_steps = [t0]
        self.ParaV = None
        self.state = VehicleState.PENDING


class VehicleGeneratorBase(metaclass=ABCMeta):
    """docstring for VehicleGeneratorBase"""
    def __init__(self):
        self.schedule = deque()
        self.buildSchedule()
        # self.tmCalculator()

    @abstractmethod
    def buildSchedule(self):
        pass

    def getAtTime(self, ctime):
        vehicle_to_add = []
        while self.hasVehicle() and self.schedule[0].t0 <= ctime:
            vehicle_to_add.append(self.schedule.popleft())
        return vehicle_to_add

    def tmCalculator(self):
        for i, v in enumerate(self.schedule):
            v.tm = config.case1['tm_0'] if v.vehicle.ID == 0 \
                else Helper.getTm(v, self.schedule[i-1])

    def hasVehicle(self):
        return len(self.schedule) > 0

    def FIFOIDAssigner(self):
        self.schedule = deque(sorted(self.schedule, key=lambda x:x.t0))
        for i, ove in enumerate(self.schedule):
            ove.vehicle.ID = i


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
            self.schedule.append(OnBoardVehicle(v, t))
        # self.schedule.sort(key=lambda x: x.vehicle.ID)


class Case2VehicleGenerator(VehicleGeneratorBase):
    """docstring for Case1VehicleGenerator"""
    def __init__(self):
        super(Case2VehicleGenerator, self).__init__()

    def buildSchedule(self):
        tnum = config.case2['total_cars']
        t0_lane1 = np.random.rand(tnum//2) * 40.0 + 10.0
        t0_lane2 = np.random.rand(tnum//2) * 40.0 + 10.0
        for i in range(tnum):
            v = VehicleBuilder(-1)\
                .setSpeed(config.case2['speed'])\
                .setPosition(-config.control_len)\
                .setAcceleration(0)\
                .setLane(i % 2).build()
            t = t0_lane1[i//2] if (i % 2 == 0) else t0_lane2[i//2]
            self.schedule.append(OnBoardVehicle(v, t))
        self.FIFOIDAssigner()


class Case3VehicleGenerator(VehicleGeneratorBase):
    """docstring for Case1VehicleGenerator"""
    def __init__(self, tnum):
        self.tnum = tnum
        super(Case3VehicleGenerator, self).__init__()

    def buildSchedule(self):
        tnum = self.tnum
        t0_lane1 = np.random.rand(tnum//2) * 40.0 + 10.0
        t0_lane2 = np.random.rand(tnum//2) * 40.0 + 10.0
        for i in range(tnum):
            v = VehicleBuilder(-1)\
                .setSpeed(config.case2['speed'])\
                .setPosition(-config.control_len)\
                .setAcceleration(0)\
                .setLane(i % 2).build()
            t = t0_lane1[i//2] if (i % 2 == 0) else t0_lane2[i//2]
            self.schedule.append(OnBoardVehicle(v, t))
        self.FIFOIDAssigner()


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
            v.tm = config.case1['tm_0'] if v.vehicle.ID == 0 \
                else Helper.getTm(v, self.on_board_vehicles[-1])
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

        p = gridplot([[s1, s2, s3]])
        try:
            show(p)
        except Exception as e:
            pass
        show(p)


def main():
    vehicle_generator = Case2VehicleGenerator()
    game = GameLoop(vehicle_generator)
    game.play()
    game.draw_result("result.html")


if __name__ == '__main__':
    main()