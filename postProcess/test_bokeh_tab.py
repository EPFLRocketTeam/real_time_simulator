from bokeh.plotting import figure, curdoc
from bokeh.resources import CDN
from bokeh.embed import file_html
from bokeh.io import show
from bokeh.layouts import gridplot, layout, row, column
from bokeh.models import CheckboxGroup, CustomJS, ColumnDataSource
from bokeh.models import Button, RadioGroup, FileInput, TextInput, RangeSlider, Slider, Panel, Tabs
from bokeh.themes import Theme


import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from scipy import interpolate

import time


p1 = figure()
p1.circle([1, 2, 3, 4, 5], [6, 7, 2, 4, 5], size=20, color="navy", alpha=0.5)
tab1 = Panel(child=p1, title="circle")

p2 = figure()
p2.line([1, 2, 3, 4, 5], [6, 7, 2, 4, 5], line_width=3, color="navy", alpha=0.5)
tab2 = Panel(child=p2, title="line")

show(Tabs(tabs=[tab1, tab2], height = 100, width = 100, sizing_mode = "fixed"))
#show(Tabs(tabs=[tab1, tab2]))