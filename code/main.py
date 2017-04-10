import numpy as np

from bokeh.io import curdoc
from bokeh.layouts import widgetbox, row, column, gridplot
from bokeh.models import ColumnDataSource, Select, Slider, Button
from bokeh.plotting import figure
from bokeh.palettes import Spectral6

from sklearn import cluster, datasets
from sklearn.neighbors import kneighbors_graph
from sklearn.preprocessing import StandardScaler

from game import Case3VehicleGenerator, GameLoop

def run_game():
    vehicle_generator = Case3VehicleGenerator(nvs)
    ggame = GameLoop(vehicle_generator)
    ggame.play()
    return ggame
    # game.draw_result("result.html")

def update_plot(ggame):
    ggame = run_game()
    for i, tv in enumerate(ggame.finished_vehicels):
        source[i] = ColumnDataSource(data=dict(x=tv.time_steps, y=tv.position_history))

nvs = 30

# set up plot (styling in theme.yaml)
plot = figure(toolbar_location=None, title="Merging Lane Simulation")
ggame = run_game()
for i, tv in enumerate(ggame.finished_vehicels):
    source[i] = ColumnDataSource(data=dict(x=tv.time_steps, y=tv.position_history))
    plot.line(x='x', y='y', source=source[i])

vehicle_num_slider = Slider(title="Number of vehicles",
                        value=30.0,
                        start=4.0,
                        end=100.0,
                        step=2,
                        width=400)

run_button = Button(label="Run", button_type="success")
run_button.callback = update_plot

def update_samples_or_dataset(attrname, old, new):
    nvs = int(vehicle_num_slider.value)

vehicle_num_slider.on_change('value', update_samples_or_dataset)

# set up layout
inputs = column(widgetbox(vehicle_num_slider, run_button))

# add to document
curdoc().add_root(row(inputs, plot))
curdoc().title = "Clustering"