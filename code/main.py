import numpy as np

from bokeh.io import curdoc
from bokeh.layouts import widgetbox, row, column, gridplot
from bokeh.models import ColumnDataSource, Select, Slider, Button
from bokeh.plotting import figure
from bokeh.palettes import Spectral6

from game import APPVehicleGenerator, GameLoop
from helper import Helper
from common import config

def run_game(nvs, id_assigner, min_pass_time):
    vehicle_generator = APPVehicleGenerator(nvs, id_assigner, min_pass_time)
    ggame = GameLoop(vehicle_generator)
    ggame.play()
    all_x = []; all_p = []; all_s = []; all_a = []; colors = []
    for i, tv in enumerate(ggame.finished_vehicels):
        all_x.append(tv.time_steps)
        all_p.append(tv.position_history)
        all_s.append(tv.speed_history)
        all_a.append(tv.acc_history)
        colors.append('red' if tv.vehicle.lane == 1 else 'blue')
    return dict(x=all_x, p=all_p, s=all_s, a=all_a, colors=colors)
    # game.draw_result("result.html")

def convert_id_assigner(s):
    if s == 'Main road first':
        return 'main'
    return s

def convert_limit(s):
    if s == 'None':
        return 0
    elif s == 'Max speed':
        return Helper.getTmPossSpeed(config.case1['speed'],
            float(speed_limit_slider.value) - 0.6, config.case1['speed'])
    elif s == 'Max speed & acceleration':
        tm_speed_limit = Helper.getTmPossSpeed(config.case1['speed'],
            float(speed_limit_slider.value) - 1.0, config.case1['speed'])
        tm_acc_limit = Helper.getTmPossAcc(config.case1['speed'],
            float(acc_limit_slider.value) - 0.1)
        return max(tm_speed_limit, tm_acc_limit)
    assert False, "Selected limitation method not implemented"

def update_plot():
    source.data = run_game(int(vehicle_num_slider.value),
        convert_id_assigner(id_assigner_ticker.value),
        convert_limit(limit_ticker.value))

def set_grid(p):
    p.xgrid.minor_grid_line_color = 'navy'
    p.xgrid.minor_grid_line_alpha = 0.1
    p.ygrid.minor_grid_line_color = 'navy'
    p.ygrid.minor_grid_line_alpha = 0.1

# set up plot (styling in theme.yaml)
source = ColumnDataSource(data=run_game(30, 'FIFO', 0))
TOOLS = "pan,wheel_zoom,box_zoom,reset,save,box_select,lasso_select"
plot_p = figure(tools=TOOLS, title="Position", x_axis_label="time", y_axis_label="position(m)",
    plot_width=400, plot_height=400)
plot_p.multi_line(xs='x', ys='p', color='colors', source=source)
set_grid(plot_p)
plot_s = figure(toolbar_location=None, title="Speed", x_axis_label="time", y_axis_label="speed(m/s)",
    x_range=plot_p.x_range, plot_width=400, plot_height=400)
plot_s.multi_line(xs='x', ys='s', color='colors', source=source)
set_grid(plot_s)
plot_a = figure(toolbar_location=None, title="Acceleration", x_axis_label="time",
    y_axis_label="acceleration(m/s^2)", x_range=plot_p.x_range, plot_width=400, plot_height=400)
plot_a.multi_line(xs='x', ys='a', color='colors', source=source)
set_grid(plot_a)

vehicle_num_slider = Slider(title="Number of vehicles",
    value=30.0, start=4.0, end=100.0, step=2)
speed_limit_slider = Slider(title="Speed limit",
    value=30.0, start=16.0, end=32.0, step=0.5)
acc_limit_slider = Slider(title="Acceleration limit",
    value=5.0, start=1.0, end=10.0, step=0.2)
id_assigner_ticker = Select(title="ID Assign Method",
    value='FIFO', options=['FIFO', 'Main road first'])
limit_ticker = Select(title="Limitations",
    value='None', options=['None', 'Max speed', 'Max speed & acceleration'])
run_button = Button(label="Run", button_type="success")
run_button.on_click(update_plot)

# set up layout
inputs = column(widgetbox(vehicle_num_slider, speed_limit_slider,
    acc_limit_slider, id_assigner_ticker, limit_ticker, run_button))

# add to document
layout = column(row(inputs, plot_p), row(plot_s, plot_a))
curdoc().add_root(layout)
curdoc().title = "Merge Lane Simulation"