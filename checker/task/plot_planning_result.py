
import bokeh.plotting as bkp
from bokeh.events import Tap
from bokeh.models import ColumnDataSource, WheelZoomTool, HoverTool, TapTool, CustomJS


from bokeh.io import output_notebook, push_notebook

from IPython.core.display import display, HTML
from checker.lib.load_rotate import coord_transformer
from checker.lib.load_struct import load_car_params_patch_parking








display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

ego_local_x_vec, ego_local_y_vec, _ = load_car_params_patch_parking()
coord_tf = coord_transformer()

data_slot = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_other_slot = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_obs = ColumnDataSource(data = {'x_vec':[], 'y_vec':[]})
data_path = ColumnDataSource(data = {'x_vec':[], 'y_vec':[], 'theta_vec':[]})


fig1 = bkp.figure(width=1200, height=800, match_aspect = True, aspect_scale=1)
fig1.x_range.flipped = False
fig1.outline_line_color = "black"
fig1.outline_line_width = 1.0

# measure tool
source = ColumnDataSource(data=dict(x=[], y=[]))
fig1.circle('x', 'y', size=10, source=source, color='red', legend_label='measure tool')
line_source = ColumnDataSource(data=dict(x=[], y=[]))
fig1.line('x', 'y', source=source, line_width=3, line_color = 'pink', line_dash = 'solid', legend_label='measure tool')
text_source = ColumnDataSource(data=dict(x=[], y=[], text=[]))
fig1.text('x', 'y', 'text', source=text_source, text_color='red', text_align='center', text_font_size='15pt', legend_label='measure tool')
# Define the JavaScript callback code
callback_code = """
    var x = cb_obj.x;
    var y = cb_obj.y;

    source.data['x'].push(x);
    source.data['y'].push(y);

    if (source.data['x'].length > 2) {
        source.data['x'].shift();
        source.data['y'].shift();
        source.data['x'].shift();
        source.data['y'].shift();
    }
    source.change.emit();

    if (source.data['x'].length >= 2) {
        var x1 = source.data['x'][source.data['x'].length - 2];
        var y1 = source.data['y'][source.data['y'].length - 2];
        var x2 = x;
        var y2 = y;
        var x3 = (x1 + x2) / 2;
        var y3 = (y1 + y2) / 2;

        var distance = Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));

        console.log("Distance between the last two points: " + distance);

        distance = distance.toFixed(4);
        text_source.data = {'x': [x3], 'y': [y3], 'text': [distance]};
        text_source.change.emit();

        line_source.data = {'x': [x1, x2], 'y': [y1, y2]};
        line_source.change.emit();
    }

    if (source.data['x'].length == 1) {
        text_source.data['x'].shift();
        text_source.data['y'].shift();
        text_source.data['text'].shift();
    }
    text_source.change.emit();
"""
# Create a CustomJS callback with the defined code
callback = CustomJS(args=dict(source=source, line_source=line_source, text_source=text_source), code=callback_code)
# Attach the callback to the Tap event on the plot
fig1.js_on_event(Tap, callback)


fig1.line('x_vec','y_vec',source =data_path,  line_width = 3.0, line_color = 'green', line_dash = 'solid',legend_label = 'Car Path', visible = True)

fig1.line('x_vec','y_vec',source =data_slot,  line_width = 2.0, line_color = 'black', line_dash = 'solid',legend_label = 'slot', visible = True)

fig1.multi_line('x_vec','y_vec',source =data_other_slot,  line_width = 2.0, line_color = 'black', line_dash = 'solid',legend_label = 'slot', visible = True)

fig1.scatter("x_vec", "y_vec", source=data_obs, size=3, color='grey',legend_label = 'External obstacles')


fig1.legend.label_text_font = "Times New Roman"  # 设置图例字体类型
fig1.legend.label_text_font_size = '14pt'  # 设置图例字体大小
fig1.legend.location = 'top_left'

fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)
fig1.legend.click_policy = 'hide'

data = 



